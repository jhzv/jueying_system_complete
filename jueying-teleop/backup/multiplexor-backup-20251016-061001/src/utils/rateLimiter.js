class RateLimiterWS {
    constructor(maxRequests = 100, windowMs = 60000) {
        this.maxRequests = maxRequests;
        this.windowMs = windowMs;
        this.clients = new Map();
    }

    checkLimit(clientId) {
        const now = Date.now();
        
        if (!this.clients.has(clientId)) {
            this.clients.set(clientId, {
                count: 1,
                resetTime: now + this.windowMs
            });
            return true;
        }

        const clientData = this.clients.get(clientId);

        if (now > clientData.resetTime) {
            clientData.count = 1;
            clientData.resetTime = now + this.windowMs;
            return true;
        }

        if (clientData.count >= this.maxRequests) {
            return false;
        }

        clientData.count++;
        return true;
    }

    reset(clientId) {
        this.clients.delete(clientId);
    }
}

module.exports = RateLimiterWS;
