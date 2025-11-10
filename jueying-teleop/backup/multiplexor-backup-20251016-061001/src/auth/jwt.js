const jwt = require('jsonwebtoken');
const bcrypt = require('bcrypt');
// const redis = require('redis'); // DESHABILITADO TEMPORALMENTE
const logger = require('../utils/logger');

class AuthService {
    constructor() {
        // Redis deshabilitado temporalmente
        this.redisClient = null;
        
        this.users = {
            'admin': {
                password: '$2b$10$c46gZaxj4m8zj6PbMgcaWu1O2cEDWqF/.n2w.T/iHI8Zp1Z4kho5u',
                role: 'admin',
                permissions: ['control', 'monitor', 'admin']
            },
            'operator1': {
                password: '$2b$10$wYUQEe8i8JxdCUs3QQhzQ.Hn4x1FeTDEY5rnJO6h5YjHeE2FYnjwu',
                role: 'operator',
                permissions: ['control', 'monitor']
            }
        };
    }

    async generateTokens(username, role) {
        const payload = { username, role };
        
        const accessToken = jwt.sign(
            payload,
            process.env.JWT_SECRET,
            { expiresIn: process.env.JWT_EXPIRE }
        );
        
        const refreshToken = jwt.sign(
            payload,
            process.env.JWT_SECRET,
            { expiresIn: process.env.JWT_REFRESH_EXPIRE }
        );
        
        // Redis deshabilitado temporalmente
        // if (this.redisClient) {
        //     await this.redisClient.setEx(
        //         `refresh:${username}`,
        //         7 * 24 * 60 * 60,
        //         refreshToken
        //     );
        // }
        
        return { accessToken, refreshToken };
    }

    async login(username, password) {
        const user = this.users[username];
        if (!user) {
            throw new Error('Usuario no encontrado');
        }
        
        const valid = await bcrypt.compare(password, user.password);
        if (!valid) {
            throw new Error('Contraseña incorrecta');
        }
        
        logger.info(`Login exitoso: ${username}`);
        return await this.generateTokens(username, user.role);
    }

    async refreshAccessToken(refreshToken) {
        try {
            const decoded = jwt.verify(refreshToken, process.env.JWT_SECRET);
            
            const accessToken = jwt.sign(
                { username: decoded.username, role: decoded.role },
                process.env.JWT_SECRET,
                { expiresIn: process.env.JWT_EXPIRE }
            );
            
            return accessToken;
        } catch (error) {
            throw new Error('Refresh token expirado o inválido');
        }
    }

    verifyToken(token) {
        return jwt.verify(token, process.env.JWT_SECRET);
    }

    async logout(username) {
        logger.info(`Logout: ${username}`);
    }

    async registerUser(username, password, role = 'operator') {
        if (this.users[username]) {
            throw new Error('Usuario ya existe');
        }
        
        const hashedPassword = await bcrypt.hash(password, 10);
        this.users[username] = {
            password: hashedPassword,
            role,
            permissions: role === 'admin' ? ['control', 'monitor', 'admin'] : ['control', 'monitor']
        };
        
        logger.info(`Usuario registrado: ${username} (${role})`);
        return { username, role };
    }
}

module.exports = new AuthService();
