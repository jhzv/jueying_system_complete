// src/auth/jwt-manager.js
// Gestión de tokens JWT para autenticación

const jwt = require('jsonwebtoken');
const bcrypt = require('bcrypt');
const users = require('../../config/users');
const logger = require('../utils/logger');

const JWT_SECRET = process.env.JWT_SECRET || 'default-secret-change-in-production';
const JWT_EXPIRATION = process.env.JWT_EXPIRATION || '24h';

class JWTManager {
  /**
   * Autentica un usuario y genera un token
   * @param {string} username 
   * @param {string} password 
   * @returns {Object|null} { token, user } o null si falla
   */
  async authenticate(username, password) {
    try {
      const user = Object.values(users.users).find(u => u.username === username);
      
      if (!user) {
        logger.warn(`Authentication failed: user not found - ${username}`);
        return null;
      }

      const isValid = await bcrypt.compare(password, user.passwordHash);
      
      if (!isValid) {
        logger.warn(`Authentication failed: invalid password - ${username}`);
        return null;
      }

      const token = this.generateToken({
        username: user.username,
        role: user.role,
        permissions: user.permissions
      });

      logger.info(`User authenticated: ${username} (${user.role})`);

      return {
        token,
        user: {
          username: user.username,
          role: user.role,
          permissions: user.permissions
        }
      };
    } catch (error) {
      logger.error(`Authentication error: ${error.message}`);
      return null;
    }
  }

  /**
   * Genera un token JWT
   * @param {Object} payload 
   * @returns {string} Token JWT
   */
  generateToken(payload) {
    return jwt.sign(payload, JWT_SECRET, {
      expiresIn: JWT_EXPIRATION,
      issuer: 'jueying-multiplexor'
    });
  }

  /**
   * Verifica y decodifica un token
   * @param {string} token 
   * @returns {Object|null} Payload decodificado o null si inválido
   */
  verifyToken(token) {
    try {
      const decoded = jwt.verify(token, JWT_SECRET, {
        issuer: 'jueying-multiplexor'
      });
      return decoded;
    } catch (error) {
      if (error.name === 'TokenExpiredError') {
        logger.warn('Token expired');
      } else if (error.name === 'JsonWebTokenError') {
        logger.warn('Invalid token');
      } else {
        logger.error(`Token verification error: ${error.message}`);
      }
      return null;
    }
  }

  /**
   * Verifica si un usuario tiene un permiso específico
   * @param {Object} decodedToken 
   * @param {string} permission 
   * @returns {boolean}
   */
  hasPermission(decodedToken, permission) {
    if (!decodedToken || !decodedToken.permissions) {
      return false;
    }
    return decodedToken.permissions.includes(permission);
  }

  /**
   * Genera hash de una contraseña (útil para crear nuevos usuarios)
   * @param {string} password 
   * @returns {Promise<string>}
   */
  async hashPassword(password) {
    return await bcrypt.hash(password, 10);
  }
}

module.exports = new JWTManager();
