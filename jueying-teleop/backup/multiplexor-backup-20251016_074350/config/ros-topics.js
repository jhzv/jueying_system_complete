// config/ros-topics.js
// Configuración de tópicos ROS del robot Jueying Lite3
// ✅ TÓPICOS VERIFICADOS Y CORREGIDOS

module.exports = {
  // COMANDOS (Publicar desde interfaz → Robot)
  command: {
    velocity: {
      topic: '/cmd_vel',
      messageType: 'geometry_msgs/Twist',
      description: 'Control de velocidad lineal y angular',
      rateLimit: 20 // Hz
    },
    simple: {
      topic: '/simple_cmd',
      messageType: 'message_transformer/SimpleCMD',
      description: 'Comandos discretos (stand, sit, etc)',
      rateLimit: 5 // Hz
    },
    complex: {
      topic: '/complex_cmd',
      messageType: 'message_transformer/ComplexCMD',
      description: 'Comandos con parámetros flotantes',
      rateLimit: 5 // Hz
    }
  },

  // FEEDBACK (Suscribirse desde Robot → Interfaz)
  feedback: {
    odometry: {
      topic: '/leg_odom',
      messageType: 'nav_msgs/Odometry',
      description: 'Odometría calculada por las patas',
      broadcastRate: 10 // Hz
    },
    imu: {
      topic: '/imu/data',
      messageType: 'sensor_msgs/Imu',
      description: 'Datos de IMU (acelerómetro, giroscopio)',
      broadcastRate: 50 // Hz
    },
    joints: {
      topic: '/joint_states',
      messageType: 'sensor_msgs/JointState',
      description: 'Estado de todas las articulaciones',
      broadcastRate: 20 // Hz
    },
    handle: {
      topic: '/handle_state',
      messageType: 'message_transformer/HandleState',
      description: 'Estado del handle y sistema',
      broadcastRate: 10 // Hz
    }
  },

  // MAPEO DE COMANDOS DE INTERFAZ → MENSAJES ROS
  commandMapping: {
    // Comandos simples
    'stand': {
      type: 'simple',
      data: { data: 'stand' }
    },
    'sit': {
      type: 'simple',
      data: { data: 'sit' }
    },
    'damping': {
      type: 'simple',
      data: { data: 'damping' }
    },
    'walk': {
      type: 'simple',
      data: { data: 'walk' }
    },
    'trot': {
      type: 'simple',
      data: { data: 'trot' }
    },
    'run': {
      type: 'simple',
      data: { data: 'run' }
    },
    
    // Comandos de movimiento (velocity)
    'move': {
      type: 'velocity',
      // Los valores se pasan dinámicamente
      defaultData: {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      }
    },
    'stop': {
      type: 'velocity',
      data: {
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      }
    },
    
    // Comandos complejos (con parámetros)
    'body_height': {
      type: 'complex',
      cmd: 'body_height'
      // value se pasa dinámicamente
    },
    'pitch': {
      type: 'complex',
      cmd: 'pitch'
    },
    'roll': {
      type: 'complex',
      cmd: 'roll'
    },
    'yaw': {
      type: 'complex',
      cmd: 'yaw'
    }
  }
};
