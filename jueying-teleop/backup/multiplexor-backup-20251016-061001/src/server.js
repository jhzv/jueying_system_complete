require('dotenv').config();
const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const helmet = require('helmet');
const cors = require('cors');
const rateLimit = require('express-rate-limit');

const authService = require('./auth/jwt');
const { authenticateToken, requireAdmin } = require('./auth/middleware');
const controlManager = require('./control/manager');
const rosValidator = require('./ros/validator');
const logger = require('./utils/logger');
const RateLimiterWS = require('./utils/rateLimiter');

const PORT = process.env.PORT || 8080;
const ROSBRIDGE_URL = process.env.ROSBRIDGE_URL || 'ws://localhost:9090';

const app = express();
app.use(helmet());
app.use(cors());
app.use(express.json());

const apiLimiter = rateLimit({
    windowMs: 15 * 60 * 1000,
    max: 100
});
app.use('/api/', apiLimiter);

app.get('/api/health', (req, res) => {
    res.json({
        status: 'ok',
        rosbridge: rosbridgeConnected,
        clients: clients.size,
        uptime: process.uptime()
    });
});

app.post('/api/auth/login', async (req, res) => {
    try {
        const { username, password } = req.body;
        const tokens = await authService.login(username, password);
        res.json({ 
            success: true, 
            ...tokens,
            user: { username, role: authService.users[username].role }
        });
    } catch (error) {
        logger.error(`Login fallido: ${error.message}`);
        res.status(401).json({ error: error.message });
    }
});

app.post('/api/auth/refresh', async (req, res) => {
    try {
        const { refreshToken } = req.body;
        const accessToken = await authService.refreshAccessToken(refreshToken);
        res.json({ accessToken });
    } catch (error) {
        res.status(403).json({ error: error.message });
    }
});

app.post('/api/auth/logout', authenticateToken, async (req, res) => {
    await authService.logout(req.user.username);
    res.json({ success: true });
});

app.get('/api/status', authenticateToken, (req, res) => {
    res.json({
        rosbridge: rosbridgeConnected,
        clients: Array.from(clients.values()).map(c => ({
            id: c.id,
            username: c.username,
            connected: c.ws.readyState === WebSocket.OPEN
        })),
        control: controlManager.getStatus()
    });
});

const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

const clients = new Map();
let rosbridgeWs = null;
let rosbridgeConnected = false;
const wsRateLimiter = new RateLimiterWS();

function connectToRosbridge() {
    logger.info(`Conectando a rosbridge: ${ROSBRIDGE_URL}`);
    
    rosbridgeWs = new WebSocket(ROSBRIDGE_URL);
    
    rosbridgeWs.on('open', () => {
        rosbridgeConnected = true;
        logger.info('Conectado a rosbridge');
        broadcastToAll({ type: 'ros_connection', connected: true });
    });
    
    rosbridgeWs.on('message', (data) => {
        try {
            const rosMessage = JSON.parse(data);
            broadcastToAll({
                type: 'ros_data',
                topic: rosMessage.topic,
                msg: rosMessage.msg
            });
        } catch (error) {
            logger.error(`Error mensaje ROS: ${error.message}`);
        }
    });
    
    rosbridgeWs.on('close', () => {
        rosbridgeConnected = false;
        logger.warn('Desconectado rosbridge. Reintentando...');
        broadcastToAll({ type: 'ros_connection', connected: false });
        setTimeout(connectToRosbridge, 3000);
    });
    
    rosbridgeWs.on('error', (error) => {
        logger.error(`Error rosbridge: ${error.message}`);
    });
}

wss.on('connection', (ws, req) => {
    const clientId = generateClientId();
    logger.info(`Nueva conexión: ${clientId}`);
    
    clients.set(clientId, {
        id: clientId,
        ws: ws,
        authenticated: false,
        username: null,
        lastHeartbeat: Date.now()
    });
    
    ws.on('message', async (data) => {
        try {
            const message = JSON.parse(data);
            await handleClientMessage(clientId, message);
        } catch (error) {
            logger.error(`Error mensaje ${clientId}: ${error.message}`);
            sendToClient(clientId, { type: 'error', error: error.message });
        }
    });
    
    ws.on('close', () => handleClientDisconnect(clientId));
    ws.on('error', (error) => logger.error(`Error cliente ${clientId}: ${error.message}`));
    
    ws.send(JSON.stringify({ type: 'connected', clientId, requireAuth: true }));
});

async function handleClientMessage(clientId, message) {
    const client = clients.get(clientId);
    if (!client) return;
    
    if (!wsRateLimiter.checkLimit(clientId)) {
        sendToClient(clientId, { type: 'error', error: 'Rate limit excedido' });
        return;
    }
    
    switch (message.type) {
        case 'auth':
            await handleAuth(clientId, message.token);
            break;
        case 'heartbeat':
            client.lastHeartbeat = Date.now();
            sendToClient(clientId, { type: 'heartbeat_ack' });
            break;
        case 'control_request':
            handleControlRequest(clientId, message.action);
            break;
        case 'ros_command':
            handleROSCommand(clientId, message);
            break;
    }
}

async function handleAuth(clientId, token) {
    try {
        const user = authService.verifyToken(token);
        const client = clients.get(clientId);
        
        client.authenticated = true;
        client.username = user.username;
        client.role = user.role;
        
        logger.info(`Cliente ${clientId} autenticado como ${user.username}`);
        sendToClient(clientId, {
            type: 'auth_success',
            username: user.username,
            role: user.role
        });
    } catch (error) {
        logger.error(`Auth fallida ${clientId}: ${error.message}`);
        sendToClient(clientId, { type: 'auth_failed', error: 'Token inválido' });
        setTimeout(() => clients.get(clientId)?.ws.close(), 1000);
    }
}

function handleControlRequest(clientId, action) {
    const client = clients.get(clientId);
    
    if (!client?.authenticated) {
        sendToClient(clientId, { type: 'error', error: 'No autenticado' });
        return;
    }
    
    if (action === 'request') {
        const result = controlManager.requestControl(clientId, client.username);
        
        if (result.granted) {
            sendToClient(clientId, { type: 'control_granted' });
            broadcastToOthers(clientId, { type: 'control_taken', by: client.username });
        } else {
            sendToClient(clientId, {
                type: 'control_denied',
                reason: `Control en uso por ${result.holder}`,
                position: result.position
            });
        }
    } else if (action === 'release') {
        if (controlManager.releaseControl(clientId)) {
            sendToClient(clientId, { type: 'control_released' });
            broadcastToAll({ type: 'control_available' });
        }
    }
}

function handleROSCommand(clientId, message) {
    const client = clients.get(clientId);
    
    if (!client?.authenticated) {
        sendToClient(clientId, { type: 'error', error: 'No autenticado' });
        return;
    }
    
    if (!controlManager.hasControl(clientId)) {
        sendToClient(clientId, { type: 'error', error: 'Sin control' });
        return;
    }
    
    const { topic, msg, msgType } = message;
    
    try {
        const sanitizedTopic = rosValidator.sanitizeTopic(topic);
        const validation = rosValidator.validate(sanitizedTopic, msg);
        
        if (!validation.valid) {
            sendToClient(clientId, { type: 'error', error: `Validación: ${validation.error}` });
            return;
        }
        
        if (rosbridgeWs?.readyState === WebSocket.OPEN) {
            rosbridgeWs.send(JSON.stringify({
                op: 'publish',
                topic: sanitizedTopic,
                type: msgType,
                msg: validation.message
            }));
            
            controlManager.updateCommandTimestamp(clientId);
            logger.debug(`Comando ROS: ${sanitizedTopic}`);
        } else {
            sendToClient(clientId, { type: 'error', error: 'ROS no conectado' });
        }
    } catch (error) {
        logger.error(`Error comando ROS: ${error.message}`);
        sendToClient(clientId, { type: 'error', error: error.message });
    }
}

function handleClientDisconnect(clientId) {
    const client = clients.get(clientId);
    
    if (client) {
        logger.info(`Desconectado: ${client.username || clientId}`);
        
        if (controlManager.releaseControl(clientId)) {
            broadcastToAll({ type: 'control_available' });
        }
        
        controlManager.removeFromQueue(clientId);
        clients.delete(clientId);
    }
}

setInterval(() => {
    const holder = controlManager.getStatus().holder;
    
    if (holder && controlManager.checkSafetyTimeout(holder.id)) {
        logger.warn(`Safety timeout: ${holder.username}`);
        
        if (rosbridgeWs?.readyState === WebSocket.OPEN) {
            rosbridgeWs.send(JSON.stringify({
                op: 'publish',
                topic: '/cmd_vel',
                type: 'geometry_msgs/Twist',
                msg: {
                    linear: { x: 0, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: 0 }
                }
            }));
        }
    }
}, 100);

function sendToClient(clientId, message) {
    const client = clients.get(clientId);
    if (client?.ws.readyState === WebSocket.OPEN) {
        client.ws.send(JSON.stringify(message));
    }
}

function broadcastToAll(message) {
    const data = JSON.stringify(message);
    clients.forEach(client => {
        if (client.ws.readyState === WebSocket.OPEN) {
            client.ws.send(data);
        }
    });
}

function broadcastToOthers(excludeId, message) {
    const data = JSON.stringify(message);
    clients.forEach((client, id) => {
        if (id !== excludeId && client.ws.readyState === WebSocket.OPEN) {
            client.ws.send(data);
        }
    });
}

function generateClientId() {
    return `client_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}

server.listen(PORT, () => {
    logger.info('='.repeat(50));
    logger.info(`Multiplexor Seguro ACTIVO`);
    logger.info(`Puerto: ${PORT}`);
    logger.info(`ROS Bridge: ${ROSBRIDGE_URL}`);
    logger.info('='.repeat(50));
    
    connectToRosbridge();
});

process.on('SIGINT', () => {
    logger.info('Cerrando servidor...');
    
    if (rosbridgeWs?.readyState === WebSocket.OPEN) {
        rosbridgeWs.send(JSON.stringify({
            op: 'publish',
            topic: '/cmd_vel',
            type: 'geometry_msgs/Twist',
            msg: { linear: {x:0,y:0,z:0}, angular: {x:0,y:0,z:0} }
        }));
    }
    
    wss.close();
    server.close();
    if (rosbridgeWs) rosbridgeWs.close();
    process.exit(0);
});
