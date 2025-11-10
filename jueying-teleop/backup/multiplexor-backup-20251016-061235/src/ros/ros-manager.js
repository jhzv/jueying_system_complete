// src/ros/ros-manager.js
// Gestor de conexión ROS y publicación/suscripción de tópicos
// ✅ CORREGIDO: Usa tópicos correctos del Jueying Lite3

const ROSLIB = require('roslib');
const rosConfig = require('../../config/ros-topics');
const logger = require('../utils/logger');

class ROSManager {
  constructor(rosbridgeUrl) {
    this.rosbridgeUrl = rosbridgeUrl;
    this.ros = null;
    this.publishers = {};
    this.subscribers = {};
    this.connected = false;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 10;
    this.reconnectDelay = 5000; // 5 segundos
    
    // Callbacks para telemetría
    this.telemetryCallbacks = [];
  }

  /**
   * Conecta a rosbridge
   */
  async connect() {
    return new Promise((resolve, reject) => {
      logger.info(`Connecting to rosbridge at ${this.rosbridgeUrl}`);

      this.ros = new ROSLIB.Ros({
        url: this.rosbridgeUrl
      });

      this.ros.on('connection', () => {
        logger.ros('connected', { url: this.rosbridgeUrl });
        this.connected = true;
        this.reconnectAttempts = 0;
        this.initializePublishers();
        this.initializeSubscribers();
        resolve();
      });

      this.ros.on('error', (error) => {
        logger.error(`ROS connection error: ${error}`);
        this.connected = false;
        if (this.reconnectAttempts === 0) {
          reject(error);
        }
      });

      this.ros.on('close', () => {
        logger.warn('ROS connection closed');
        this.connected = false;
        this.attemptReconnect();
      });
    });
  }

  /**
   * Intenta reconectar a rosbridge
   */
  attemptReconnect() {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      logger.error('Max reconnection attempts reached. Giving up.');
      return;
    }

    this.reconnectAttempts++;
    logger.info(`Attempting to reconnect (${this.reconnectAttempts}/${this.maxReconnectAttempts})...`);

    setTimeout(() => {
      this.connect().catch(err => {
        logger.error(`Reconnection failed: ${err.message}`);
      });
    }, this.reconnectDelay);
  }

  /**
   * Inicializa publishers para comandos
   */
  initializePublishers() {
    // Publisher para cmd_vel (velocidades)
    this.publishers.velocity = new ROSLIB.Topic({
      ros: this.ros,
      name: rosConfig.command.velocity.topic,
      messageType: rosConfig.command.velocity.messageType
    });

    // Publisher para simple_cmd (comandos discretos)
    this.publishers.simple = new ROSLIB.Topic({
      ros: this.ros,
      name: rosConfig.command.simple.topic,
      messageType: rosConfig.command.simple.messageType
    });

    // Publisher para complex_cmd (comandos con parámetros)
    this.publishers.complex = new ROSLIB.Topic({
      ros: this.ros,
      name: rosConfig.command.complex.topic,
      messageType: rosConfig.command.complex.messageType
    });

    logger.ros('publishers_initialized', {
      topics: Object.keys(this.publishers)
    });
  }

  /**
   * Inicializa subscribers para feedback del robot
   */
  initializeSubscribers() {
    // Suscripción a odometría
    this.subscribers.odometry = new ROSLIB.Topic({
      ros: this.ros,
      name: rosConfig.feedback.odometry.topic,
      messageType: rosConfig.feedback.odometry.messageType
    });

    this.subscribers.odometry.subscribe((message) => {
      this.broadcastTelemetry('odometry', message);
    });

    // Suscripción a IMU
    this.subscribers.imu = new ROSLIB.Topic({
      ros: this.ros,
      name: rosConfig.feedback.imu.topic,
      messageType: rosConfig.feedback.imu.messageType
    });

    this.subscribers.imu.subscribe((message) => {
      this.broadcastTelemetry('imu', message);
    });

    // Suscripción a estados de juntas
    this.subscribers.joints = new ROSLIB.Topic({
      ros: this.ros,
      name: rosConfig.feedback.joints.topic,
      messageType: rosConfig.feedback.joints.messageType
    });

    this.subscribers.joints.subscribe((message) => {
      this.broadcastTelemetry('joints', message);
    });

    // Suscripción a handle state
    this.subscribers.handle = new ROSLIB.Topic({
      ros: this.ros,
      name: rosConfig.feedback.handle.topic,
      messageType: rosConfig.feedback.handle.messageType
    });

    this.subscribers.handle.subscribe((message) => {
      this.broadcastTelemetry('handle', message);
    });

    logger.ros('subscribers_initialized', {
      topics: Object.keys(this.subscribers)
    });
  }

  /**
   * Publica un comando al robot
   * @param {string} commandName - Nombre del comando (ej: 'stand', 'move')
   * @param {Object} data - Datos del comando
   */
  publishCommand(commandName, data = {}) {
    if (!this.connected) {
      throw new Error('Not connected to ROS');
    }

    const commandConfig = rosConfig.commandMapping[commandName];
    
    if (!commandConfig) {
      throw new Error(`Unknown command: ${commandName}`);
    }

    const publisher = this.publishers[commandConfig.type];
    
    if (!publisher) {
      throw new Error(`No publisher for command type: ${commandConfig.type}`);
    }

    let message;

    switch (commandConfig.type) {
      case 'simple':
        message = new ROSLIB.Message(commandConfig.data);
        break;

      case 'velocity':
        message = new ROSLIB.Message(data.velocity || commandConfig.data);
        break;

      case 'complex':
        message = new ROSLIB.Message({
          cmd: commandConfig.cmd,
          value: data.value || 0
        });
        break;

      default:
        throw new Error(`Unsupported command type: ${commandConfig.type}`);
    }

    publisher.publish(message);
    
    logger.command(commandName, data.user || 'unknown', {
      type: commandConfig.type,
      topic: publisher.name
    });
  }

  /**
   * Registra un callback para telemetría
   * @param {Function} callback - Función que recibe (topic, data)
   */
  onTelemetry(callback) {
    this.telemetryCallbacks.push(callback);
  }

  /**
   * Difunde datos de telemetría a todos los callbacks
   * @param {string} topic 
   * @param {Object} data 
   */
  broadcastTelemetry(topic, data) {
    logger.telemetry(topic, JSON.stringify(data).length);
    
    this.telemetryCallbacks.forEach(callback => {
      try {
        callback(topic, data);
      } catch (error) {
        logger.error(`Error in telemetry callback: ${error.message}`);
      }
    });
  }

  /**
   * Verifica el estado de la conexión
   * @returns {Object} Estado de conexión y topics
   */
  getStatus() {
    return {
      connected: this.connected,
      publishers: Object.keys(this.publishers).map(key => ({
        name: key,
        topic: this.publishers[key].name
      })),
      subscribers: Object.keys(this.subscribers).map(key => ({
        name: key,
        topic: this.subscribers[key].name
      })),
      telemetryCallbacks: this.telemetryCallbacks.length
    };
  }

  /**
   * Desconecta de ROS y limpia recursos
   */
  disconnect() {
    if (this.ros) {
      // Unsubscribe de todos los topics
      Object.values(this.subscribers).forEach(sub => {
        sub.unsubscribe();
      });

      this.ros.close();
      this.connected = false;
      logger.ros('disconnected', {});
    }
  }
}

module.exports = ROSManager;
