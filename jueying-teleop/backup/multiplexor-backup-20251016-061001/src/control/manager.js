const EventEmitter = require('events');
const logger = require('../utils/logger');

class ControlManager extends EventEmitter {
    constructor() {
        super();
        this.controlHolder = null;
        this.controlQueue = [];
        this.lastCommandTime = {};
        this.safetyTimeout = parseInt(process.env.SAFETY_TIMEOUT_MS) || 250;
    }

    requestControl(clientId, username) {
        if (this.controlHolder === null) {
            this.grantControl(clientId, username);
            return { granted: true, position: 0 };
        } else if (this.controlHolder.id === clientId) {
            return { granted: true, position: 0 };
        } else {
            const position = this.controlQueue.length + 1;
            this.controlQueue.push({ clientId, username, timestamp: Date.now() });
            logger.info(`Cliente ${username} en cola, posición ${position}`);
            return { granted: false, position, holder: this.controlHolder.username };
        }
    }

    grantControl(clientId, username) {
        this.controlHolder = { id: clientId, username, since: Date.now() };
        this.lastCommandTime[clientId] = Date.now();
        logger.info(`Control otorgado a ${username} (${clientId})`);
        this.emit('control_granted', { clientId, username });
    }

    releaseControl(clientId) {
        if (this.controlHolder && this.controlHolder.id === clientId) {
            const released = this.controlHolder;
            this.controlHolder = null;
            logger.info(`Control liberado por ${released.username}`);
            this.emit('control_released', { clientId });
            
            if (this.controlQueue.length > 0) {
                const next = this.controlQueue.shift();
                this.grantControl(next.clientId, next.username);
            }
            
            return true;
        }
        return false;
    }

    hasControl(clientId) {
        return this.controlHolder && this.controlHolder.id === clientId;
    }

    updateCommandTimestamp(clientId) {
        this.lastCommandTime[clientId] = Date.now();
    }

    checkSafetyTimeout(clientId) {
        const lastCmd = this.lastCommandTime[clientId] || 0;
        return (Date.now() - lastCmd) > this.safetyTimeout;
    }

    getStatus() {
        return {
            holder: this.controlHolder,
            queue: this.controlQueue.map(c => ({ 
                username: c.username, 
                waiting: Date.now() - c.timestamp 
            }))
        };
    }

    removeFromQueue(clientId) {
        this.controlQueue = this.controlQueue.filter(c => c.clientId !== clientId);
    }
}

module.exports = new ControlManager();
