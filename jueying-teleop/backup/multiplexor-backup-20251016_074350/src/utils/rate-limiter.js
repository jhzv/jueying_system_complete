// src/utils/rate-limiter.js
// Control de frecuencia de comandos por usuario/sesión

class RateLimiter {
  constructor(maxRate = 20) {
    this.maxRate = maxRate; // Comandos por segundo
    this.windows = new Map(); // clientId -> { count, resetTime }
    this.windowDuration = 1000; // 1 segundo
  }

  /**
   * Verifica si un cliente puede enviar un comando
   * @param {string} clientId - ID del cliente
   * @returns {boolean} true si puede enviar, false si está limitado
   */
  checkLimit(clientId) {
    const now = Date.now();
    const window = this.windows.get(clientId);

    if (!window || now >= window.resetTime) {
      // Nueva ventana
      this.windows.set(clientId, {
        count: 1,
        resetTime: now + this.windowDuration
      });
      return true;
    }

    if (window.count < this.maxRate) {
      window.count++;
      return true;
    }

    return false; // Rate limit excedido
  }

  /**
   * Limpia ventanas expiradas (llamar periódicamente)
   */
  cleanup() {
    const now = Date.now();
    for (const [clientId, window] of this.windows.entries()) {
      if (now >= window.resetTime + 5000) { // 5 segundos de gracia
        this.windows.delete(clientId);
      }
    }
  }

  /**
   * Elimina la ventana de un cliente específico
   * @param {string} clientId 
   */
  reset(clientId) {
    this.windows.delete(clientId);
  }

  /**
   * Obtiene estadísticas actuales
   * @returns {Object} Estadísticas de rate limiting
   */
  getStats() {
    return {
      activeClients: this.windows.size,
      maxRate: this.maxRate,
      windowDuration: this.windowDuration
    };
  }
}

module.exports = RateLimiter;
