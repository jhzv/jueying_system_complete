/**
 * Constantes del Protocolo UDP - Jueying Lite3
 * 
 * Basado en protocol.h del repositorio oficial
 * Referencia: Jueying Lite3 Motion Host Communication Interface V1.0.6-0
 */

const UDP_COMMANDS = {
  // ============================================================
  // MODOS DE OPERACIÓN
  // ============================================================
  MODE_POSE: 0x21010D05,   // Modo poses/acciones
  MODE_MOVE: 0x21010D06,   // Modo movimiento continuo
  
  // ============================================================
  // ACCIONES (solo en modo POSE)
  // ============================================================
  ACTION_SIT_STAND: 0x21010202,    // Sentarse/Levantarse (toggle)
  ACTION_SAY_HELLO: 0x21010507,    // Saludar
  ACTION_LONG_JUMP: 0x21010508,    // Salto largo
  ACTION_TWIST_JUMP: 0x21010502,   // Salto con giro
  ACTION_MOONWALK: 0x21010506,     // Moonwalk
  ACTION_TWIST: 0x21010500,        // Giro en su lugar
  ACTION_ZERO: 0x21010501,         // Posición cero/neutral
  ACTION_STAND: 0x21010202,        // Pararse
  ACTION_SIT: 0x21010206,          // Sentarse (damping mode)
  
  // ============================================================
  // CONTROL DE EJES DE MOVIMIENTO (solo en modo MOVE)
  // ============================================================
  AXIS_TRANSLATION_FB: 0x21010130,  // Adelante/Atrás (-32767 a 32767)
  AXIS_TRANSLATION_LR: 0x21010131,  // Izquierda/Derecha (-32767 a 32767)
  AXIS_ROTATION: 0x21010135,        // Rotación (-32767 a 32767)
  
  // ============================================================
  // GAITS (modos de marcha) (solo en modo MOVE)
  // ============================================================
  // Terreno plano
  GAIT_FLAT_SLOW: 0x21010300,       // Marcha lenta
  GAIT_FLAT_MEDIUM: 0x21010307,     // Marcha media (default)
  GAIT_FLAT_FAST: 0x21010303,       // Marcha rápida
  GAIT_FLAT_CRAWL: 0x21010308,      // Gateo
  
  // Terreno rugoso
  GAIT_RUG_GRIP: 0x21010304,        // Agarre en terreno rugoso
  GAIT_RUG_GENERAL: 0x21010305,     // General para terreno rugoso
  GAIT_RUG_HSTEP: 0x21010306,       // Paso alto
  
  // ============================================================
  // COMANDOS DE SISTEMA
  // ============================================================
  HEARTBEAT: 0x21040001,            // Keep-alive (debe enviarse a 4Hz)
  EMERGENCY_STOP: 0x21010A00,       // Paro de emergencia
  SAVE_DATA: 0x21030001,            // Guardar datos (soft emergency stop)
  
  // ============================================================
  // BODY ADJUSTMENT (opcional - verificar soporte)
  // ============================================================
  BODY_HEIGHT: 0x21010140,          // Altura del cuerpo
  BODY_PITCH: 0x21010141,           // Inclinación pitch
  BODY_ROLL: 0x21010142,            // Inclinación roll
  BODY_YAW: 0x21010143,             // Rotación yaw
};

/**
 * Mapeo de nombres legibles a códigos
 */
const COMMAND_NAMES = {
  // Modos
  'mode_pose': UDP_COMMANDS.MODE_POSE,
  'mode_move': UDP_COMMANDS.MODE_MOVE,
  
  // Acciones
  'sit_stand': UDP_COMMANDS.ACTION_SIT_STAND,
  'say_hello': UDP_COMMANDS.ACTION_SAY_HELLO,
  'long_jump': UDP_COMMANDS.ACTION_LONG_JUMP,
  'twist_jump': UDP_COMMANDS.ACTION_TWIST_JUMP,
  'moonwalk': UDP_COMMANDS.ACTION_MOONWALK,
  'twist': UDP_COMMANDS.ACTION_TWIST,
  'zero': UDP_COMMANDS.ACTION_ZERO,
  'stand': UDP_COMMANDS.ACTION_STAND,
  'sit': UDP_COMMANDS.ACTION_SIT,
  
  // Ejes
  'forward_back': UDP_COMMANDS.AXIS_TRANSLATION_FB,
  'left_right': UDP_COMMANDS.AXIS_TRANSLATION_LR,
  'rotation': UDP_COMMANDS.AXIS_ROTATION,
  
  // Gaits
  'flat_slow': UDP_COMMANDS.GAIT_FLAT_SLOW,
  'flat_medium': UDP_COMMANDS.GAIT_FLAT_MEDIUM,
  'flat_fast': UDP_COMMANDS.GAIT_FLAT_FAST,
  'flat_crawl': UDP_COMMANDS.GAIT_FLAT_CRAWL,
  'rug_grip': UDP_COMMANDS.GAIT_RUG_GRIP,
  'rug_general': UDP_COMMANDS.GAIT_RUG_GENERAL,
  'rug_hstep': UDP_COMMANDS.GAIT_RUG_HSTEP,
  
  // Sistema
  'heartbeat': UDP_COMMANDS.HEARTBEAT,
  'emergency_stop': UDP_COMMANDS.EMERGENCY_STOP,
  'save_data': UDP_COMMANDS.SAVE_DATA,
};

/**
 * Obtener nombre del comando desde código
 */
function getCommandName(code) {
  for (const [name, value] of Object.entries(COMMAND_NAMES)) {
    if (value === code) {
      return name;
    }
  }
  return `UNKNOWN_0x${code.toString(16)}`;
}

/**
 * Validar valor de eje
 */
function validateAxisValue(value) {
  return Math.max(-32767, Math.min(32767, Math.floor(value)));
}

/**
 * Convertir porcentaje (-100 a 100) a valor de eje
 */
function percentToAxisValue(percent) {
  const clamped = Math.max(-100, Math.min(100, percent));
  return Math.round((clamped / 100) * 32767);
}

/**
 * Convertir valor de eje a porcentaje
 */
function axisValueToPercent(value) {
  const clamped = Math.max(-32767, Math.min(32767, value));
  return Math.round((clamped / 32767) * 100);
}

/**
 * Limites de valores
 */
const LIMITS = {
  AXIS_MIN: -32767,
  AXIS_MAX: 32767,
  PERCENT_MIN: -100,
  PERCENT_MAX: 100,
  
  // Frecuencias recomendadas
  HEARTBEAT_HZ: 4,         // 4Hz = 250ms
  AXIS_SEND_HZ: 20,        // 20Hz = 50ms
  COMMAND_MIN_INTERVAL: 50 // Mínimo 50ms entre comandos
};

/**
 * Configuraciones por defecto
 */
const DEFAULT_CONFIG = {
  MOTION_HOST_IP: '192.168.1.120',
  MOTION_HOST_PORT: 43893,
  PERCEPTION_HOST_IP: '192.168.1.103',
  
  HEARTBEAT_INTERVAL: 250,  // ms
  AXIS_SEND_INTERVAL: 50,   // ms
  RECONNECT_DELAY: 1000,    // ms
  
  DEFAULT_GAIT: 'flat_medium',
  DEFAULT_MODE: 'pose'
};

module.exports = {
  UDP_COMMANDS,
  COMMAND_NAMES,
  LIMITS,
  DEFAULT_CONFIG,
  
  // Funciones helper
  getCommandName,
  validateAxisValue,
  percentToAxisValue,
  axisValueToPercent
};
