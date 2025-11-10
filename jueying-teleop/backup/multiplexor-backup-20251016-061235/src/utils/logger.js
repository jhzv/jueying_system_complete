// src/utils/logger.js
// Sistema de logging con Winston

const winston = require('winston');
const path = require('path');

const logLevel = process.env.LOG_LEVEL || 'info';
const logFile = process.env.LOG_FILE || '/tmp/multiplexor.log';

const logger = winston.createLogger({
  level: logLevel,
  format: winston.format.combine(
    winston.format.timestamp({ format: 'YYYY-MM-DD HH:mm:ss' }),
    winston.format.errors({ stack: true }),
    winston.format.splat(),
    winston.format.json()
  ),
  defaultMeta: { service: 'jueying-multiplexor' },
  transports: [
    // Archivo de logs
    new winston.transports.File({ 
      filename: logFile,
      maxsize: 10485760, // 10MB
      maxFiles: 5
    }),
    // Consola con formato legible
    new winston.transports.Console({
      format: winston.format.combine(
        winston.format.colorize(),
        winston.format.printf(({ timestamp, level, message, ...meta }) => {
          let msg = `${timestamp} [${level}]: ${message}`;
          if (Object.keys(meta).length > 0 && meta.service !== 'jueying-multiplexor') {
            msg += ` ${JSON.stringify(meta)}`;
          }
          return msg;
        })
      )
    })
  ]
});

// Wrapper para logging de eventos específicos del robot
logger.command = (command, user, data) => {
  logger.info('COMMAND', { command, user, data });
};

logger.telemetry = (topic, dataSize) => {
  logger.debug('TELEMETRY', { topic, dataSize });
};

logger.connection = (event, clientId, details) => {
  logger.info('CONNECTION', { event, clientId, ...details });
};

logger.ros = (event, details) => {
  logger.info('ROS', { event, ...details });
};

module.exports = logger;
