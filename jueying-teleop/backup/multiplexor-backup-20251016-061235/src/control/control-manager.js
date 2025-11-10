// src/control/control-manager.js
// Gestión de sesiones de control y prioridades

const logger = require('../utils/logger');

class ControlManager {
  constructor() {
    this.activeControllers = new Map(); // clientId -> { user, timestamp, priority }
    this.currentController = null;
    this.controlTimeout = 30000; // 30 segundos de inactividad
    this.checkInterval = null;
  }

  /**
   * Inicia el gestor de control
   */
  start() {
    // Verificar timeouts cada 5 segundos
    this.checkInterval = setInterval(() => {
      this.checkTimeouts();
    }, 5000);
    
    logger.info('Control manager started');
  }

  /**
   * Detiene el gestor de control
   */
  stop() {
    if (this.checkInterval) {
      clearInterval(this.checkInterval);
      this.checkInterval = null;
    }
    logger.info('Control manager stopped');
  }

  /**
   * Registra un cliente como potencial controlador
   * @param {string} clientId 
   * @param {Object} user - { username, role, permissions }
   */
  registerClient(clientId, user) {
    const priority = this.calculatePriority(user.role);
    
    this.activeControllers.set(clientId, {
      user,
      timestamp: Date.now(),
      priority,
      commandCount: 0
    });

    logger.connection('client_registered', clientId, {
      username: user.username,
      role: user.role,
      priority
    });

    this.evaluateControl();
  }

  /**
   * Desregistra un cliente
   * @param {string} clientId 
   */
  unregisterClient(clientId) {
    const wasController = this.currentController === clientId;
    
    this.activeControllers.delete(clientId);
    
    if (wasController) {
      this.currentController = null;
      this.evaluateControl();
    }

    logger.connection('client_unregistered', clientId, {
      wasController
    });
  }

  /**
   * Actualiza la actividad de un cliente
   * @param {string} clientId 
   */
  updateActivity(clientId) {
    const client = this.activeControllers.get(clientId);
    if (client) {
      client.timestamp = Date.now();
      client.commandCount++;
    }
  }

  /**
   * Verifica si un cliente puede enviar comandos
   * @param {string} clientId 
   * @returns {Object} { allowed, reason }
   */
  canControl(clientId) {
    const client = this.activeControllers.get(clientId);
    
    if (!client) {
      return { allowed: false, reason: 'not_registered' };
    }

    if (!client.user.permissions.includes('control')) {
      return { allowed: false, reason: 'no_permission' };
    }

    if (this.currentController === null) {
      this.currentController = clientId;
      logger.info(`Control granted to ${client.user.username}`);
    }

    if (this.currentController !== clientId) {
      // Verificar si el cliente actual tiene mayor prioridad
      const currentClient = this.activeControllers.get(this.currentController);
      
      if (!currentClient || client.priority > currentClient.priority) {
        logger.info(`Control transferred from ${currentClient?.user.username || 'unknown'} to ${client.user.username}`);
        this.currentController = clientId;
      } else {
        return { 
          allowed: false, 
          reason: 'another_controller',
          controller: currentClient.user.username
        };
      }
    }

    return { allowed: true };
  }

  /**
   * Calcula la prioridad basada en el rol
   * @param {string} role 
   * @returns {number} Prioridad (mayor = más prioritario)
   */
  calculatePriority(role) {
    const priorities = {
      'admin': 100,
      'operator': 50,
      'viewer': 0
    };
    return priorities[role] || 0;
  }

  /**
   * Reevalúa quién debe tener el control
   */
  evaluateControl() {
    if (this.activeControllers.size === 0) {
      this.currentController = null;
      return;
    }

    // Si no hay controlador actual, asignar al de mayor prioridad
    if (!this.currentController || !this.activeControllers.has(this.currentController)) {
      let highestPriority = -1;
      let bestCandidate = null;

      for (const [clientId, client] of this.activeControllers) {
        if (client.user.permissions.includes('control') && client.priority > highestPriority) {
          highestPriority = client.priority;
          bestCandidate = clientId;
        }
      }

      if (bestCandidate) {
        this.currentController = bestCandidate;
        const client = this.activeControllers.get(bestCandidate);
        logger.info(`Control assigned to ${client.user.username}`);
      }
    }
  }

  /**
   * Verifica timeouts de inactividad
   */
  checkTimeouts() {
    const now = Date.now();
    const timedOutClients = [];

    for (const [clientId, client] of this.activeControllers) {
      if (now - client.timestamp > this.controlTimeout) {
        timedOutClients.push(clientId);
      }
    }

    timedOutClients.forEach(clientId => {
      logger.warn(`Client timed out: ${clientId}`);
      this.unregisterClient(clientId);
    });
  }

  /**
   * Obtiene el estado actual del control
   * @returns {Object}
   */
  getStatus() {
    const currentControllerData = this.currentController 
      ? this.activeControllers.get(this.currentController)
      : null;

    return {
      currentController: currentControllerData ? {
        username: currentControllerData.user.username,
        role: currentControllerData.user.role,
        commandCount: currentControllerData.commandCount,
        activeTime: Date.now() - currentControllerData.timestamp
      } : null,
      activeClients: Array.from(this.activeControllers.entries()).map(([id, data]) => ({
        id,
        username: data.user.username,
        role: data.user.role,
        priority: data.priority,
        commandCount: data.commandCount
      }))
    };
  }

  /**
   * Libera el control forzadamente
   * @param {string} adminClientId - ID del admin que solicita liberar
   */
  releaseControl(adminClientId) {
    const admin = this.activeControllers.get(adminClientId);
    
    if (!admin || admin.user.role !== 'admin') {
      return { success: false, reason: 'not_authorized' };
    }

    this.currentController = null;
    this.evaluateControl();

    logger.info(`Control released by admin ${admin.user.username}`);
    return { success: true };
  }
}

module.exports = ControlManager;
