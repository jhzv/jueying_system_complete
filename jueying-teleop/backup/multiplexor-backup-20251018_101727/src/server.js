/**
 * Multiplexor Jueying Lite3 v2.1 - Híbrido ROS + UDP
 * 
 * Servidor WebSocket que combina:
 * - UDP directo para comandos al Motion Host
 * - ROS/rosbridge para telemetría
 * - Autenticación JWT
 * - Control de sesiones
 */

require('dotenv').config();

const WebSocket = require('ws');
const ROSLIB = require('roslib');
const jwt = require('jsonwebtoken');
const logger = require('./utils/logger');
const UDPClient = require('./udp/client');
const { UDP_COMMANDS } = require('./udp/constants');
const users = require('../config/users');

// ============================================================
// CONFIGURACIÓN
// ============================================================
const CONFIG = {
  port: parseInt(process.env.PORT) || 8080,
  rosbridgeUrl: process.env.ROSBRIDGE_URL || 'ws://localhost:9090',
  jwtSecret: process.env.JWT_SECRET || 'changeme',
  maxClients: parseInt(process.env.MAX_CLIENTS) || 5,
  movementTimeout: parseInt(process.env.MOVEMENT_TIMEOUT) || 250,
  commandTimeout: parseInt(process.env.COMMAND_TIMEOUT) || 1000
};

// ============================================================
// CLIENTES GLOBALES
// ============================================================
let rosClient = null;
let udpClient = null;

// Subscriptores ROS para telemetría
const rosSubscribers = {
  odometry: null,
  imu: null,
  joints: null,
  handle: null
};

// ============================================================
// ESTADO DEL SERVIDOR
// ============================================================
const serverState = {
  startTime: Date.now(),
  clients: new Map(), // clientId -> { ws, user, lastActivity }
  currentController: null, // usuario con control activo
  lastTelemetry: {
    odometry: null,
    imu: null,
    joints: null,
    handle: null
  }
};

// ============================================================
// INICIALIZAR ROS CLIENT
// ============================================================
function initROS() {
  return new Promise((resolve, reject) => {
    logger.info(`[ROS] Connecting to rosbridge: ${CONFIG.rosbridgeUrl}`);
    
    rosClient = new ROSLIB.Ros({
      url: CONFIG.rosbridgeUrl
    });
    
    rosClient.on('connection', () => {
      logger.info('[ROS] Connected to rosbridge');
      
      // Suscribirse a tópicos de telemetría
      subscribeToROS();
      
      resolve();
    });
    
    rosClient.on('error', (error) => {
      logger.error('[ROS] Connection error:', error);
      reject(error);
    });
    
    rosClient.on('close', () => {
      logger.warn('[ROS] Connection closed. Attempting reconnect...');
      setTimeout(initROS, 5000);
    });
  });
}

/**
 * Suscribirse a tópicos ROS de telemetría
 */
function subscribeToROS() {
  // Odometría
  rosSubscribers.odometry = new ROSLIB.Topic({
    ros: rosClient,
    name: '/leg_odom',
    messageType: 'nav_msgs/Odometry'
  });
  
  rosSubscribers.odometry.subscribe((message) => {
    serverState.lastTelemetry.odometry = message;
    broadcastTelemetry('odometry', message);
  });
  
  // IMU
  rosSubscribers.imu = new ROSLIB.Topic({
    ros: rosClient,
    name: '/imu/data',
    messageType: 'sensor_msgs/Imu'
  });
  
  rosSubscribers.imu.subscribe((message) => {
    serverState.lastTelemetry.imu = message;
    broadcastTelemetry('imu', message);
  });
  
  // Joint States
  rosSubscribers.joints = new ROSLIB.Topic({
    ros: rosClient,
    name: '/joint_states',
    messageType: 'sensor_msgs/JointState'
  });
  
  rosSubscribers.joints.subscribe((message) => {
    serverState.lastTelemetry.joints = message;
    broadcastTelemetry('joints', message);
  });
  
  // Handle State (si existe)
  rosSubscribers.handle = new ROSLIB.Topic({
    ros: rosClient,
    name: '/handle_state',
    messageType: 'std_msgs/String' // Ajustar según tipo real
  });
  
  rosSubscribers.handle.subscribe((message) => {
    serverState.lastTelemetry.handle = message;
    broadcastTelemetry('handle', message);
  });
  
  logger.info('[ROS] Subscribed to telemetry topics');
}

// ============================================================
// INICIALIZAR UDP CLIENT
// ============================================================
async function initUDP() {
  logger.info('[UDP] Initializing client...');
  
  udpClient = new UDPClient({
    host: process.env.ROBOT_MOTION_HOST || '192.168.1.120',
    port: parseInt(process.env.ROBOT_MOTION_PORT) || 43893,
    heartbeatInterval: parseInt(process.env.HEARTBEAT_INTERVAL) || 250,
    debug: process.env.DEBUG_UDP === 'true'
  });
  
  udpClient.on('error', (error) => {
    logger.error('[UDP] Error:', error);
  });
  
  udpClient.on('disconnected', () => {
    logger.warn('[UDP] Disconnected. Attempting reconnect...');
    setTimeout(() => udpClient.connect(), 5000);
  });
  
  await udpClient.connect();
  
  // Inicializar en modo POSE por defecto
  await udpClient.setMode('pose');
  
  logger.info('[UDP] Client initialized');
}

// ============================================================
// WEBSOCKET SERVER
// ============================================================
const wss = new WebSocket.Server({ port: CONFIG.port });

wss.on('connection', (ws, req) => {
  const clientId = generateClientId();
  const clientIp = req.socket.remoteAddress;
  
  logger.info(`[WS] New connection: ${clientId} from ${clientIp}`);
  
  // Estado inicial del cliente
  serverState.clients.set(clientId, {
    ws,
    user: null,
    authenticated: false,
    lastActivity: Date.now()
  });
  
  // Verificar límite de clientes
  if (serverState.clients.size > CONFIG.maxClients) {
    ws.send(JSON.stringify({
      type: 'error',
      message: 'Server full. Maximum clients reached.'
    }));
    ws.close();
    serverState.clients.delete(clientId);
    return;
  }
  
  // Manejar mensajes
  ws.on('message', async (data) => {
    try {
      const message = JSON.parse(data);
      await handleMessage(clientId, message);
    } catch (error) {
      logger.error(`[WS] Message error from ${clientId}:`, error);
      ws.send(JSON.stringify({
        type: 'error',
        message: 'Invalid message format'
      }));
    }
  });
  
  // Manejar desconexión
  ws.on('close', () => {
    logger.info(`[WS] Client disconnected: ${clientId}`);
    
    const client = serverState.clients.get(clientId);
    
    // Si tenía control, liberarlo
    if (serverState.currentController?.clientId === clientId) {
      serverState.currentController = null;
      logger.info('[CONTROL] Control released');
      
      // Detener robot por seguridad
      if (udpClient) {
        udpClient.stop();
      }
      
      // Notificar a otros clientes
      broadcastMessage({
        type: 'control_released',
        message: 'Controller disconnected'
      });
    }
    
    serverState.clients.delete(clientId);
  });
  
  // Enviar mensaje de bienvenida
  ws.send(JSON.stringify({
    type: 'connected',
    clientId,
    message: 'Welcome to Jueying Lite3 Multiplexor v2.1'
  }));
});

// ============================================================
// MANEJADOR DE MENSAJES WEBSOCKET
// ============================================================
async function handleMessage(clientId, message) {
  const client = serverState.clients.get(clientId);
  if (!client) return;
  
  const { ws, user, authenticated } = client;
  
  // Actualizar actividad
  client.lastActivity = Date.now();
  
  logger.debug(`[WS] Message from ${clientId}: ${message.type}`);
  
  switch (message.type) {
    case 'auth':
      await handleAuth(clientId, message);
      break;
      
    case 'request_control':
      await handleRequestControl(clientId);
      break;
      
    case 'release_control':
      await handleReleaseControl(clientId);
      break;
      
    case 'command':
      if (!authenticated) {
        ws.send(JSON.stringify({
          type: 'error',
          message: 'Authentication required'
        }));
        return;
      }
      await handleCommand(clientId, message);
      break;
      
    case 'status':
      await handleStatusRequest(clientId);
      break;
      
    case 'ping':
      ws.send(JSON.stringify({ type: 'pong', timestamp: Date.now() }));
      break;
      
    default:
      ws.send(JSON.stringify({
        type: 'error',
        message: `Unknown message type: ${message.type}`
      }));
  }
}

// ============================================================
// AUTENTICACIÓN
// ============================================================
async function handleAuth(clientId, message) {
  const client = serverState.clients.get(clientId);
  const { username, password } = message;
  
  logger.info(`[AUTH] Login attempt: ${username}`);
  
  // Verificar credenciales
  const user = await users.authenticate(username, password);
  
  if (!user) {
    logger.warn(`[AUTH] Failed login: ${username}`);
    client.ws.send(JSON.stringify({
      type: 'auth_failed',
      message: 'Invalid credentials'
    }));
    return;
  }
  
  // Generar token JWT
  const token = jwt.sign(
    { username: user.username, role: user.role },
    CONFIG.jwtSecret,
    { expiresIn: process.env.JWT_EXPIRATION || '24h' }
  );
  
  // Actualizar cliente
  client.user = user;
  client.authenticated = true;
  
  logger.info(`[AUTH] User authenticated: ${username} (${user.role})`);
  
  // Responder con éxito
  client.ws.send(JSON.stringify({
    type: 'auth_success',
    token,
    user: {
      username: user.username,
      role: user.role,
      permissions: user.permissions
    }
  }));
  
  // Enviar estado actual
  await handleStatusRequest(clientId);
}

// ============================================================
// CONTROL DE SESIONES
// ============================================================
async function handleRequestControl(clientId) {
  const client = serverState.clients.get(clientId);
  
  if (!client.authenticated) {
    client.ws.send(JSON.stringify({
      type: 'error',
      message: 'Authentication required'
    }));
    return;
  }
  
  // Verificar permisos
  if (!client.user.permissions.includes('control')) {
    client.ws.send(JSON.stringify({
      type: 'control_denied',
      message: 'Insufficient permissions'
    }));
    return;
  }
  
  // Si ya hay controlador
  if (serverState.currentController) {
    const currentUser = serverState.currentController.user.username;
    
    // Si es admin, puede tomar control
    if (client.user.role === 'admin') {
      logger.info(`[CONTROL] Admin ${client.user.username} taking control from ${currentUser}`);
      
      // Notificar al anterior
      const prevClient = serverState.clients.get(serverState.currentController.clientId);
      if (prevClient) {
        prevClient.ws.send(JSON.stringify({
          type: 'control_taken',
          message: `Control taken by admin: ${client.user.username}`
        }));
      }
    } else {
      client.ws.send(JSON.stringify({
        type: 'control_denied',
        message: `Control already held by: ${currentUser}`
      }));
      return;
    }
  }
  
  // Asignar control
  serverState.currentController = {
    clientId,
    user: client.user,
    since: Date.now()
  };
  
  logger.info(`[CONTROL] Control granted to: ${client.user.username}`);
  
  client.ws.send(JSON.stringify({
    type: 'control_granted',
    message: 'You now have control of the robot'
  }));
  
  // Notificar a otros clientes
  broadcastMessage({
    type: 'control_changed',
    controller: client.user.username
  }, clientId);
}

async function handleReleaseControl(clientId) {
  if (serverState.currentController?.clientId === clientId) {
    const username = serverState.currentController.user.username;
    serverState.currentController = null;
    
    // Detener robot por seguridad
    if (udpClient) {
      await udpClient.stop();
    }
    
    logger.info(`[CONTROL] Control released by: ${username}`);
    
    broadcastMessage({
      type: 'control_released',
      message: `Control released by ${username}`
    });
  }
}

// ============================================================
// MANEJADOR DE COMANDOS
// ============================================================
async function handleCommand(clientId, message) {
  const client = serverState.clients.get(clientId);
  
  // Verificar que tenga control
  if (!serverState.currentController || serverState.currentController.clientId !== clientId) {
    client.ws.send(JSON.stringify({
      type: 'error',
      message: 'You do not have control'
    }));
    return;
  }
  
  const { command, data } = message;
  
  logger.debug(`[COMMAND] ${command} from ${client.user.username}`);
  
  try {
    let success = false;
    
    switch (command) {
      // Cambio de modo
      case 'set_mode':
        success = await udpClient.setMode(data.mode);
        break;
      
      // Acciones (modo POSE)
      case 'sit_stand':
      case 'say_hello':
      case 'long_jump':
      case 'twist_jump':
      case 'moonwalk':
      case 'twist':
      case 'zero':
        success = await udpClient.executeAction(command);
        break;
      
      // Movimiento (modo MOVE)
      case 'move':
        const { forward_back, left_right, rotation } = data;
        success = await udpClient.movePercent(forward_back, left_right, rotation);
        break;
      
      case 'stop':
        success = await udpClient.stop();
        break;
      
      // Cambio de gait
      case 'set_gait':
        success = await udpClient.setGait(data.gait);
        break;
      
      // Emergencia
      case 'emergency_stop':
        success = await udpClient.emergencyStop();
        logger.warn(`[EMERGENCY] Stop triggered by ${client.user.username}`);
        break;
      
      default:
        client.ws.send(JSON.stringify({
          type: 'error',
          message: `Unknown command: ${command}`
        }));
        return;
    }
    
    // Responder
    client.ws.send(JSON.stringify({
      type: 'command_result',
      command,
      success,
      timestamp: Date.now()
    }));
    
    if (!success) {
      logger.warn(`[COMMAND] Failed: ${command}`);
    }
    
  } catch (error) {
    logger.error(`[COMMAND] Error executing ${command}:`, error);
    client.ws.send(JSON.stringify({
      type: 'error',
      message: `Command failed: ${error.message}`
    }));
  }
}

// ============================================================
// STATUS REQUEST
// ============================================================
async function handleStatusRequest(clientId) {
  const client = serverState.clients.get(clientId);
  
  const status = {
    type: 'status',
    data: {
      server: {
        uptime: Date.now() - serverState.startTime,
        clients: serverState.clients.size,
        version: '2.1.0'
      },
      ros: {
        connected: rosClient && rosClient.isConnected,
        url: CONFIG.rosbridgeUrl
      },
      udp: udpClient ? udpClient.getStatus() : null,
      control: {
        currentController: serverState.currentController ? {
          username: serverState.currentController.user.username,
          role: serverState.currentController.user.role,
          since: serverState.currentController.since
        } : null,
        hasControl: serverState.currentController?.clientId === clientId
      },
      telemetry: {
        hasOdometry: !!serverState.lastTelemetry.odometry,
        hasIMU: !!serverState.lastTelemetry.imu,
        hasJoints: !!serverState.lastTelemetry.joints
      }
    }
  };
  
  client.ws.send(JSON.stringify(status));
}

// ============================================================
// BROADCAST
// ============================================================
function broadcastTelemetry(topic, data) {
  const message = JSON.stringify({
    type: 'telemetry',
    topic,
    data,
    timestamp: Date.now()
  });
  
  serverState.clients.forEach((client) => {
    if (client.authenticated && client.ws.readyState === WebSocket.OPEN) {
      client.ws.send(message);
    }
  });
}

function broadcastMessage(message, excludeClientId = null) {
  const jsonMessage = JSON.stringify(message);
  
  serverState.clients.forEach((client, clientId) => {
    if (clientId !== excludeClientId && client.ws.readyState === WebSocket.OPEN) {
      client.ws.send(jsonMessage);
    }
  });
}

// ============================================================
// UTILIDADES
// ============================================================
function generateClientId() {
  return `client_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
}

// ============================================================
// INICIALIZACIÓN Y SHUTDOWN
// ============================================================
async function startServer() {
  try {
    logger.info('='.repeat(60));
    logger.info('Jueying Lite3 Multiplexor v2.1 - Hybrid ROS + UDP');
    logger.info('='.repeat(60));
    
    // Inicializar UDP
    await initUDP();
    
    // Inicializar ROS
    await initROS();
    
    logger.info(`[SERVER] WebSocket listening on port ${CONFIG.port}`);
    logger.info('[SERVER] Ready to accept connections');
    logger.info('='.repeat(60));
    
  } catch (error) {
    logger.error('[SERVER] Startup failed:', error);
    process.exit(1);
  }
}

async function shutdown() {
  logger.info('[SERVER] Shutting down...');
  
  // Detener robot
  if (udpClient) {
    await udpClient.stop();
    udpClient.disconnect();
  }
  
  // Cerrar ROS
  if (rosClient) {
    rosClient.close();
  }
  
  // Cerrar WebSocket
  wss.close();
  
  // Cerrar clientes
  serverState.clients.forEach((client) => {
    client.ws.close();
  });
  
  logger.info('[SERVER] Shutdown complete');
  process.exit(0);
}

// Manejadores de señales
process.on('SIGINT', shutdown);
process.on('SIGTERM', shutdown);

// Manejo de errores no capturados
process.on('uncaughtException', (error) => {
  logger.error('[SERVER] Uncaught exception:', error);
  shutdown();
});

process.on('unhandledRejection', (reason) => {
  logger.error('[SERVER] Unhandled rejection:', reason);
});

// ============================================================
// INICIAR SERVIDOR
// ============================================================
startServer();

module.exports = { wss, serverState };
