// src/server.js
// Servidor WebSocket del Multiplexor Jueying Lite3
// ✅ Versión Corregida con tópicos ROS correctos

require('dotenv').config();
const WebSocket = require('ws');
const ROSManager = require('./ros/ros-manager');
const ControlManager = require('./control/control-manager');
const JWTManager = require('./auth/jwt-manager');
const RateLimiter = require('./utils/rate-limiter');
const logger = require('./utils/logger');

// Configuración
const PORT = process.env.PORT || 8080;
const ROSBRIDGE_URL = process.env.ROSBRIDGE_URL || 'ws://localhost:9090';
const MAX_COMMAND_RATE = parseInt(process.env.MAX_COMMAND_RATE) || 20;

class MultiplexorServer {
  constructor() {
    this.wss = null;
    this.rosManager = null;
    this.controlManager = null;
    this.rateLimiter = null;
    this.clients = new Map(); // ws -> { id, user, authenticated }
  }

  /**
   * Inicia el servidor
   */
  async start() {
    try {
      logger.info('Starting Jueying Multiplexor Server...');

      // Inicializar componentes
      this.rosManager = new ROSManager(ROSBRIDGE_URL);
      this.controlManager = new ControlManager();
      this.rateLimiter = new RateLimiter(MAX_COMMAND_RATE);

      // Conectar a ROS
      await this.rosManager.connect();

      // Configurar callback de telemetría
      this.rosManager.onTelemetry((topic, data) => {
        this.broadcastTelemetry(topic, data);
      });

      // Iniciar gestor de control
      this.controlManager.start();

      // Iniciar servidor WebSocket
      this.wss = new WebSocket.Server({ port: PORT });
      this.setupWebSocketServer();

      logger.info(`✅ Multiplexor server running on port ${PORT}`);
      logger.info(`✅ Connected to ROS at ${ROSBRIDGE_URL}`);
      logger.info(`✅ System ready for connections`);

      // Cleanup periódico
      setInterval(() => {
        this.rateLimiter.cleanup();
      }, 60000); // Cada minuto

    } catch (error) {
      logger.error(`Failed to start server: ${error.message}`);
      process.exit(1);
    }
  }

  /**
   * Configura el servidor WebSocket
   */
  setupWebSocketServer() {
    this.wss.on('connection', (ws) => {
      const clientId = this.generateClientId();
      
      this.clients.set(ws, {
        id: clientId,
        user: null,
        authenticated: false,
        connectedAt: new Date()
      });

      logger.connection('new_connection', clientId, {
        total_clients: this.clients.size
      });

      ws.on('message', (data) => {
        this.handleMessage(ws, data);
      });

      ws.on('close', () => {
        this.handleDisconnect(ws);
      });

      ws.on('error', (error) => {
        logger.error(`WebSocket error for client ${clientId}: ${error.message}`);
      });

      // Enviar mensaje de bienvenida
      this.sendMessage(ws, {
        type: 'welcome',
        message: 'Connected to Jueying Multiplexor',
        clientId,
        requiresAuth: true
      });
    });
  }

  /**
   * Maneja mensajes entrantes
   */
  async handleMessage(ws, data) {
    const client = this.clients.get(ws);
    
    if (!client) {
      return;
    }

    try {
      const message = JSON.parse(data);

      // Autenticación
      if (message.type === 'auth') {
        await this.handleAuth(ws, message);
        return;
      }

      // Verificar autenticación
      if (!client.authenticated) {
        this.sendError(ws, 'not_authenticated', 'Authentication required');
        return;
      }

      // Procesar mensaje según tipo
      switch (message.type) {
        case 'command':
          this.handleCommand(ws, message);
          break;

        case 'request_control':
          this.handleRequestControl(ws);
          break;

        case 'release_control':
          this.handleReleaseControl(ws);
          break;

        case 'status':
          this.handleStatusRequest(ws);
          break;

        case 'ping':
          this.sendMessage(ws, { type: 'pong', timestamp: Date.now() });
          break;

        default:
          this.sendError(ws, 'unknown_type', `Unknown message type: ${message.type}`);
      }

    } catch (error) {
      logger.error(`Error handling message from ${client.id}: ${error.message}`);
      this.sendError(ws, 'invalid_message', error.message);
    }
  }

  /**
   * Maneja autenticación
   */
  async handleAuth(ws, message) {
    const client = this.clients.get(ws);
    
    const { username, password } = message;

    if (!username || !password) {
      this.sendError(ws, 'invalid_credentials', 'Username and password required');
      return;
    }

    const authResult = await JWTManager.authenticate(username, password);

    if (!authResult) {
      this.sendError(ws, 'auth_failed', 'Invalid credentials');
      return;
    }

    // Actualizar cliente
    client.authenticated = true;
    client.user = authResult.user;

    // Registrar en control manager
    this.controlManager.registerClient(client.id, authResult.user);

    // Enviar respuesta exitosa
    this.sendMessage(ws, {
      type: 'auth_success',
      token: authResult.token,
      user: authResult.user
    });

    logger.connection('authenticated', client.id, {
      username: authResult.user.username,
      role: authResult.user.role
    });
  }

  /**
   * Maneja comandos al robot
   */
  handleCommand(ws, message) {
    const client = this.clients.get(ws);

    // Verificar rate limit
    if (!this.rateLimiter.checkLimit(client.id)) {
      this.sendError(ws, 'rate_limit', 'Command rate limit exceeded');
      return;
    }

    // Verificar permiso de control
    const controlCheck = this.controlManager.canControl(client.id);
    
    if (!controlCheck.allowed) {
      this.sendError(ws, 'control_denied', controlCheck.reason, {
        controller: controlCheck.controller
      });
      return;
    }

    // Publicar comando a ROS
    try {
      this.rosManager.publishCommand(message.command, {
        ...message.data,
        user: client.user.username
      });

      this.controlManager.updateActivity(client.id);

      this.sendMessage(ws, {
        type: 'command_ack',
        command: message.command,
        timestamp: Date.now()
      });

    } catch (error) {
      logger.error(`Command error: ${error.message}`);
      this.sendError(ws, 'command_failed', error.message);
    }
  }

  /**
   * Maneja solicitud de control
   */
  handleRequestControl(ws) {
    const client = this.clients.get(ws);
    const controlCheck = this.controlManager.canControl(client.id);

    if (controlCheck.allowed) {
      this.sendMessage(ws, {
        type: 'control_granted',
        timestamp: Date.now()
      });
    } else {
      this.sendError(ws, 'control_denied', controlCheck.reason, {
        controller: controlCheck.controller
      });
    }
  }

  /**
   * Maneja liberación de control
   */
  handleReleaseControl(ws) {
    const client = this.clients.get(ws);
    const result = this.controlManager.releaseControl(client.id);

    if (result.success) {
      this.sendMessage(ws, {
        type: 'control_released',
        timestamp: Date.now()
      });
      
      // Notificar a todos los clientes
      this.broadcast({
        type: 'control_available',
        timestamp: Date.now()
      });
    } else {
      this.sendError(ws, 'release_failed', result.reason);
    }
  }

  /**
   * Maneja solicitud de estado
   */
  handleStatusRequest(ws) {
    this.sendMessage(ws, {
      type: 'status',
      ros: this.rosManager.getStatus(),
      control: this.controlManager.getStatus(),
      server: {
        uptime: process.uptime(),
        clients: this.clients.size,
        memory: process.memoryUsage()
      }
    });
  }

  /**
   * Maneja desconexión
   */
  handleDisconnect(ws) {
    const client = this.clients.get(ws);
    
    if (client) {
      if (client.authenticated) {
        this.controlManager.unregisterClient(client.id);
      }
      
      logger.connection('disconnected', client.id, {
        username: client.user?.username,
        duration: Date.now() - client.connectedAt.getTime()
      });
    }

    this.clients.delete(ws);
  }

  /**
   * Difunde telemetría a todos los clientes autenticados
   */
  broadcastTelemetry(topic, data) {
    const message = {
      type: 'telemetry',
      topic,
      data,
      timestamp: Date.now()
    };

    this.broadcast(message, (client) => client.authenticated);
  }

  /**
   * Envía mensaje a todos los clientes (con filtro opcional)
   */
  broadcast(message, filter = null) {
    this.wss.clients.forEach((ws) => {
      if (ws.readyState === WebSocket.OPEN) {
        const client = this.clients.get(ws);
        
        if (!filter || filter(client)) {
          ws.send(JSON.stringify(message));
        }
      }
    });
  }

  /**
   * Envía mensaje a un cliente específico
   */
  sendMessage(ws, message) {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(message));
    }
  }

  /**
   * Envía error a un cliente
   */
  sendError(ws, code, message, details = {}) {
    this.sendMessage(ws, {
      type: 'error',
      error: {
        code,
        message,
        ...details
      },
      timestamp: Date.now()
    });
  }

  /**
   * Genera ID único para cliente
   */
  generateClientId() {
    return `client_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  /**
   * Detiene el servidor
   */
  async shutdown() {
    logger.info('Shutting down multiplexor server...');

    if (this.controlManager) {
      this.controlManager.stop();
    }

    if (this.rosManager) {
      this.rosManager.disconnect();
    }

    if (this.wss) {
      this.wss.close();
    }

    logger.info('Server shutdown complete');
  }
}

// Crear e iniciar servidor
const server = new MultiplexorServer();

// Manejar señales de terminación
process.on('SIGINT', async () => {
  logger.info('Received SIGINT signal');
  await server.shutdown();
  process.exit(0);
});

process.on('SIGTERM', async () => {
  logger.info('Received SIGTERM signal');
  await server.shutdown();
  process.exit(0);
});

// Manejar errores no capturados
process.on('uncaughtException', (error) => {
  logger.error(`Uncaught exception: ${error.message}`);
  logger.error(error.stack);
});

process.on('unhandledRejection', (reason, promise) => {
  logger.error('Unhandled rejection at:', promise, 'reason:', reason);
});

// Iniciar servidor
server.start().catch((error) => {
  logger.error(`Failed to start: ${error.message}`);
  process.exit(1);
});
