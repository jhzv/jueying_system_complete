const authService = require('./jwt');
const logger = require('../utils/logger');

function authenticateToken(req, res, next) {
    const authHeader = req.headers['authorization'];
    const token = authHeader && authHeader.split(' ')[1];
    
    if (!token) {
        return res.status(401).json({ error: 'Token no proporcionado' });
    }
    
    try {
        const user = authService.verifyToken(token);
        req.user = user;
        next();
    } catch (error) {
        logger.warn(`Token inválido: ${error.message}`);
        return res.status(403).json({ error: 'Token inválido o expirado' });
    }
}

function requireAdmin(req, res, next) {
    if (req.user.role !== 'admin') {
        return res.status(403).json({ error: 'Requiere permisos de administrador' });
    }
    next();
}

module.exports = { authenticateToken, requireAdmin };
