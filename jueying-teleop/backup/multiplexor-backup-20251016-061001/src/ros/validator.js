const Joi = require('joi');
const logger = require('../utils/logger');

const topicSchemas = {
    '/cmd_vel': Joi.object({
        linear: Joi.object({
            x: Joi.number().min(-1.0).max(1.0).required(),
            y: Joi.number().valid(0).required(),
            z: Joi.number().valid(0).required()
        }).required(),
        angular: Joi.object({
            x: Joi.number().valid(0).required(),
            y: Joi.number().valid(0).required(),
            z: Joi.number().min(-1.5).max(1.5).required()
        }).required()
    }),
    
    '/pose_cmd': Joi.object({
        data: Joi.string().valid('stand', 'sit', 'laydown', 'dance', 'stretch', 'shake', 'jump').required()
    }),
    
    '/gait_cmd': Joi.object({
        data: Joi.string().valid('flat', 'rug').required()
    }),
    
    '/emergency_stop': Joi.object({
        data: Joi.boolean().required()
    })
};

class ROSValidator {
    validate(topic, message) {
        const schema = topicSchemas[topic];
        
        if (!schema) {
            logger.warn(`No hay esquema de validación para ${topic}`);
            return { valid: true, message };
        }
        
        const { error, value } = schema.validate(message);
        
        if (error) {
            logger.error(`Validación fallida para ${topic}: ${error.message}`);
            return { valid: false, error: error.message };
        }
        
        return { valid: true, message: value };
    }

    sanitizeTopic(topic) {
        if (!/^\/[a-zA-Z0-9_\/]+$/.test(topic)) {
            throw new Error('Nombre de topic inválido');
        }
        return topic;
    }
}

module.exports = new ROSValidator();
