/**
 * UDP Client para Jueying Lite3
 * Comunicación directa con el Motion Host (RK3588) usando protocolo UDP nativo
 * 
 * Basado en: lite3-local-web-control/utils/udp_client.py
 * Protocolo: Jueying Lite3 Motion Host Communication Interface V1.0.6-0
 */

const dgram = require('dgram');
const { EventEmitter } = require('events');
const logger = require('../utils/logger');
const { UDP_COMMANDS } = require('./constants');

class UDPClient extends EventEmitter {
  constructor(config = {}) {
    super();
    
    this.config = {
      host: config.host || process.env.ROBOT_MOTION_HOST || '192.168.1.120',
      port: parseInt(config.port || process.env.ROBOT_MOTION_PORT || 43893),
      heartbeatInterval: parseInt(config.heartbeatInterval || process.env.HEARTBEAT_INTERVAL || 250),
      debug: config.debug || false
    };
    
    this.socket = null;
    this.connected = false;
    
    // Estado del robot
    this.currentMode = null; // 'pose' or 'move'
    this.currentGait = null;
    
    // Control de ejes de movimiento
    this.axisValues = {
      forwardBack: 0,
      leftRight: 0,
      rotation: 0
    };
    
    // Heartbeat
    this.heartbeatTimer = null;
    this.heartbeatRunning = false;
    
    // Axis sending
    this.axisSendTimer = null;
    this.axisSendRunning = false;
    
    // Estadísticas
    this.stats = {
      commandsSent: 0,
      lastCommandTime: null,
      errors: 0
    };
  }
  
  /**
   * Conectar socket UDP
   */
  connect() {
    if (this.connected) {
      logger.warn('[UDP] Already connected');
      return Promise.resolve();
    }
    
    return new Promise((resolve, reject) => {
      try {
        this.socket = dgram.createSocket('udp4');
        
        this.socket.on('error', (err) => {
          logger.error('[UDP] Socket error:', err);
          this.stats.errors++;
          this.emit('error', err);
        });
        
        this.socket.on('close', () => {
          logger.info('[UDP] Socket closed');
          this.connected = false;
          this.emit('disconnected');
        });
        
        // UDP es connectionless, pero podemos bind para verificar
        this.socket.bind(() => {
          this.connected = true;
          logger.info(`[UDP] Client ready. Target: ${this.config.host}:${this.config.port}`);
          
          // Iniciar heartbeat automáticamente
          this.startHeartbeat();
          
          this.emit('connected');
          resolve();
        });
        
      } catch (error) {
        logger.error('[UDP] Connection error:', error);
        reject(error);
      }
    });
  }
  
  /**
   * Desconectar socket UDP
   */
  disconnect() {
    this.stopHeartbeat();
    this.stopAxisSending();
    
    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
    
    this.connected = false;
    logger.info('[UDP] Client disconnected');
  }
  
  /**
   * Enviar comando UDP al Motion Host
   * 
   * @param {number} code - Código del comando (hex)
   * @param {number} value - Valor del comando (default 0)
   * @param {number} type - Tipo del comando (default 0)
   * @returns {Promise<boolean>}
   */
  sendCommand(code, value = 0, type = 0) {
    if (!this.connected || !this.socket) {
      logger.error('[UDP] Cannot send command: not connected');
      return Promise.resolve(false);
    }
    
    return new Promise((resolve) => {
      try {
        // Crear buffer según protocolo: <3i (little-endian, 3 integers de 4 bytes)
        const buffer = Buffer.alloc(12);
        buffer.writeInt32LE(code, 0);   // Comando
        buffer.writeInt32LE(value, 4);  // Valor
        buffer.writeInt32LE(type, 8);   // Tipo
        
        this.socket.send(buffer, this.config.port, this.config.host, (err) => {
          if (err) {
            logger.error(`[UDP] Send error: ${err.message}`);
            this.stats.errors++;
            resolve(false);
          } else {
            this.stats.commandsSent++;
            this.stats.lastCommandTime = new Date();
            
            if (this.config.debug) {
              logger.debug(`[UDP] Sent: code=0x${code.toString(16)}, value=${value}, type=${type}`);
            }
            
            resolve(true);
          }
        });
        
      } catch (error) {
        logger.error('[UDP] Command error:', error);
        this.stats.errors++;
        resolve(false);
      }
    });
  }
  
  /**
   * Cambiar modo del robot
   * 
   * @param {string} mode - 'pose' o 'move'
   * @returns {Promise<boolean>}
   */
  async setMode(mode) {
    const modeMap = {
      'pose': UDP_COMMANDS.MODE_POSE,
      'move': UDP_COMMANDS.MODE_MOVE
    };
    
    if (!modeMap[mode]) {
      logger.error(`[UDP] Invalid mode: ${mode}`);
      return false;
    }
    
    const success = await this.sendCommand(modeMap[mode]);
    
    if (success) {
      this.currentMode = mode;
      logger.info(`[UDP] Mode changed to: ${mode}`);
      
      // Si cambiamos a move, detener ejes
      if (mode === 'move') {
        this.stopAxisSending();
        await this.setAxisValues(0, 0, 0);
      }
    }
    
    return success;
  }
  
  /**
   * Ejecutar acción (solo en modo POSE)
   * 
   * @param {string} action - Nombre de la acción
   * @returns {Promise<boolean>}
   */
  async executeAction(action) {
    if (this.currentMode !== 'pose') {
      logger.error('[UDP] Actions only work in POSE mode');
      return false;
    }
    
    const actionMap = {
      'sit_stand': UDP_COMMANDS.ACTION_SIT_STAND,
      'say_hello': UDP_COMMANDS.ACTION_SAY_HELLO,
      'long_jump': UDP_COMMANDS.ACTION_LONG_JUMP,
      'twist_jump': UDP_COMMANDS.ACTION_TWIST_JUMP,
      'moonwalk': UDP_COMMANDS.ACTION_MOONWALK,
      'twist': UDP_COMMANDS.ACTION_TWIST,
      'zero': UDP_COMMANDS.ACTION_ZERO,
      'stand': UDP_COMMANDS.ACTION_STAND,
      'sit': UDP_COMMANDS.ACTION_SIT,
    };
    
    if (!actionMap[action]) {
      logger.error(`[UDP] Unknown action: ${action}`);
      return false;
    }
    
    const success = await this.sendCommand(actionMap[action]);
    
    if (success) {
      logger.info(`[UDP] Action executed: ${action}`);
    }
    
    return success;
  }
  
  /**
   * Establecer valores de ejes de movimiento (solo en modo MOVE)
   * 
   * @param {number} forwardBack - Adelante/Atrás (-32767 a 32767)
   * @param {number} leftRight - Izquierda/Derecha (-32767 a 32767)
   * @param {number} rotation - Rotación (-32767 a 32767)
   * @returns {Promise<boolean>}
   */
  async setAxisValues(forwardBack, leftRight, rotation) {
    if (this.currentMode !== 'move') {
      logger.warn('[UDP] Axis control only works in MOVE mode');
      return false;
    }
    
    // Actualizar valores internos
    this.axisValues.forwardBack = Math.max(-32767, Math.min(32767, forwardBack));
    this.axisValues.leftRight = Math.max(-32767, Math.min(32767, leftRight));
    this.axisValues.rotation = Math.max(-32767, Math.min(32767, rotation));
    
    // Si todos son cero, detener envío continuo
    if (forwardBack === 0 && leftRight === 0 && rotation === 0) {
      this.stopAxisSending();
      
      // Enviar comando de stop una vez
      await this.sendCommand(UDP_COMMANDS.AXIS_TRANSLATION_FB, 0);
      await this.sendCommand(UDP_COMMANDS.AXIS_TRANSLATION_LR, 0);
      await this.sendCommand(UDP_COMMANDS.AXIS_ROTATION, 0);
      
      return true;
    }
    
    // Si hay valores, iniciar envío continuo
    if (!this.axisSendRunning) {
      this.startAxisSending();
    }
    
    return true;
  }
  
  /**
   * Mover robot con valores de porcentaje (-100 a 100)
   * 
   * @param {number} forwardBackPercent - Porcentaje adelante/atrás
   * @param {number} leftRightPercent - Porcentaje izquierda/derecha
   * @param {number} rotationPercent - Porcentaje rotación
   * @returns {Promise<boolean>}
   */
  async movePercent(forwardBackPercent, leftRightPercent, rotationPercent) {
    // Convertir porcentajes a valores de ejes
    const fb = Math.round((forwardBackPercent / 100) * 32767);
    const lr = Math.round((leftRightPercent / 100) * 32767);
    const rot = Math.round((rotationPercent / 100) * 32767);
    
    return await this.setAxisValues(fb, lr, rot);
  }
  
  /**
   * Detener movimiento
   */
  async stop() {
    return await this.setAxisValues(0, 0, 0);
  }
  
  /**
   * Cambiar gait (solo en modo MOVE)
   * 
   * @param {string} gait - Nombre del gait
   * @returns {Promise<boolean>}
   */
  async setGait(gait) {
    if (this.currentMode !== 'move') {
      logger.error('[UDP] Gait switching only works in MOVE mode');
      return false;
    }
    
    const gaitMap = {
      'flat_slow': UDP_COMMANDS.GAIT_FLAT_SLOW,
      'flat_medium': UDP_COMMANDS.GAIT_FLAT_MEDIUM,
      'flat_fast': UDP_COMMANDS.GAIT_FLAT_FAST,
      'flat_crawl': UDP_COMMANDS.GAIT_FLAT_CRAWL,
      'rug_grip': UDP_COMMANDS.GAIT_RUG_GRIP,
      'rug_general': UDP_COMMANDS.GAIT_RUG_GENERAL,
      'rug_hstep': UDP_COMMANDS.GAIT_RUG_HSTEP
    };
    
    if (!gaitMap[gait]) {
      logger.error(`[UDP] Unknown gait: ${gait}`);
      return false;
    }
    
    const success = await this.sendCommand(gaitMap[gait]);
    
    if (success) {
      this.currentGait = gait;
      logger.info(`[UDP] Gait set to: ${gait}`);
      
      // Pequeño delay para que el robot cambie de gait
      await new Promise(resolve => setTimeout(resolve, 100));
    }
    
    return success;
  }
  
  /**
   * Paro de emergencia
   */
  async emergencyStop() {
    logger.warn('[UDP] EMERGENCY STOP activated');
    
    // Detener todos los timers
    this.stopAxisSending();
    
    // Enviar comando de emergency stop
    return await this.sendCommand(UDP_COMMANDS.EMERGENCY_STOP);
  }
  
  /**
   * Iniciar envío de heartbeat (4Hz)
   */
  startHeartbeat() {
    if (this.heartbeatRunning) {
      return;
    }
    
    this.heartbeatRunning = true;
    
    this.heartbeatTimer = setInterval(() => {
      if (this.connected) {
        this.sendCommand(UDP_COMMANDS.HEARTBEAT);
      }
    }, this.config.heartbeatInterval);
    
    logger.info(`[UDP] Heartbeat started (${this.config.heartbeatInterval}ms)`);
  }
  
  /**
   * Detener envío de heartbeat
   */
  stopHeartbeat() {
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = null;
    }
    
    this.heartbeatRunning = false;
    logger.info('[UDP] Heartbeat stopped');
  }
  
  /**
   * Iniciar envío continuo de valores de ejes (20Hz)
   */
  startAxisSending() {
    if (this.axisSendRunning) {
      return;
    }
    
    this.axisSendRunning = true;
    
    this.axisSendTimer = setInterval(async () => {
      if (this.connected && this.currentMode === 'move') {
        // Enviar los 3 ejes
        await this.sendCommand(UDP_COMMANDS.AXIS_TRANSLATION_FB, this.axisValues.forwardBack);
        await this.sendCommand(UDP_COMMANDS.AXIS_TRANSLATION_LR, this.axisValues.leftRight);
        await this.sendCommand(UDP_COMMANDS.AXIS_ROTATION, this.axisValues.rotation);
      }
    }, 50); // 20Hz
    
    logger.info('[UDP] Axis sending started (20Hz)');
  }
  
  /**
   * Detener envío continuo de ejes
   */
  stopAxisSending() {
    if (this.axisSendTimer) {
      clearInterval(this.axisSendTimer);
      this.axisSendTimer = null;
    }
    
    this.axisSendRunning = false;
    
    // Reset valores
    this.axisValues = {
      forwardBack: 0,
      leftRight: 0,
      rotation: 0
    };
    
    logger.debug('[UDP] Axis sending stopped');
  }
  
  /**
   * Obtener estado del cliente
   */
  getStatus() {
    return {
      connected: this.connected,
      mode: this.currentMode,
      gait: this.currentGait,
      axisValues: { ...this.axisValues },
      heartbeatRunning: this.heartbeatRunning,
      axisSendRunning: this.axisSendRunning,
      stats: { ...this.stats },
      config: {
        host: this.config.host,
        port: this.config.port
      }
    };
  }
}

module.exports = UDPClient;
