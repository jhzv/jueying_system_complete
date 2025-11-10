// examples/client-example.js
// Ejemplo de cliente WebSocket para el multiplexor Jueying

class JueyingClient {
  constructor(url = 'ws://192.168.1.103:8080') {
    this.url = url;
    this.ws = null;
    this.authenticated = false;
    this.hasControl = false;
    this.reconnectAttempts = 0;
    this.maxReconnectAttempts = 5;
    
    // Callbacks
    this.onConnectCallback = null;
    this.onDisconnectCallback = null;
    this.onAuthSuccessCallback = null;
    this.onTelemetryCallback = null;
    this.onErrorCallback = null;
  }

  /**
   * Conecta al multiplexor
   */
  connect() {
    console.log(`Conectando a ${this.url}...`);
    
    this.ws = new WebSocket(this.url);

    this.ws.onopen = () => {
      console.log('✓ Conectado al multiplexor');
      this.reconnectAttempts = 0;
      if (this.onConnectCallback) {
        this.onConnectCallback();
      }
    };

    this.ws.onmessage = (event) => {
      this.handleMessage(event.data);
    };

    this.ws.onerror = (error) => {
      console.error('Error de WebSocket:', error);
      if (this.onErrorCallback) {
        this.onErrorCallback('connection_error', error);
      }
    };

    this.ws.onclose = () => {
      console.log('Desconectado del multiplexor');
      this.authenticated = false;
      this.hasControl = false;
      
      if (this.onDisconnectCallback) {
        this.onDisconnectCallback();
      }

      this.attemptReconnect();
    };
  }

  /**
   * Intenta reconectar
   */
  attemptReconnect() {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.error('Máximo de intentos de reconexión alcanzado');
      return;
    }

    this.reconnectAttempts++;
    console.log(`Reintentando conexión (${this.reconnectAttempts}/${this.maxReconnectAttempts})...`);

    setTimeout(() => {
      this.connect();
    }, 3000);
  }

  /**
   * Maneja mensajes entrantes
   */
  handleMessage(data) {
    try {
      const message = JSON.parse(data);

      switch (message.type) {
        case 'welcome':
          console.log('Bienvenida del servidor:', message.message);
          break;

        case 'auth_success':
          this.authenticated = true;
          console.log('✓ Autenticación exitosa');
          console.log('Usuario:', message.user);
          if (this.onAuthSuccessCallback) {
            this.onAuthSuccessCallback(message.user);
          }
          break;

        case 'control_granted':
          this.hasControl = true;
          console.log('✓ Control concedido');
          break;

        case 'control_denied':
          this.hasControl = false;
          console.log('✗ Control denegado:', message.error.message);
          if (message.error.controller) {
            console.log('  Controlador actual:', message.error.controller);
          }
          break;

        case 'telemetry':
          // Despachar telemetría al callback
          if (this.onTelemetryCallback) {
            this.onTelemetryCallback(message.topic, message.data);
          }
          break;

        case 'command_ack':
          console.log('✓ Comando confirmado:', message.command);
          break;

        case 'error':
          console.error('Error del servidor:', message.error);
          if (this.onErrorCallback) {
            this.onErrorCallback(message.error.code, message.error.message);
          }
          break;

        case 'pong':
          // Respuesta a ping
          break;

        default:
          console.log('Mensaje desconocido:', message);
      }
    } catch (error) {
      console.error('Error procesando mensaje:', error);
    }
  }

  /**
   * Autentica con el multiplexor
   */
  authenticate(username, password) {
    this.send({
      type: 'auth',
      username,
      password
    });
  }

  /**
   * Solicita control del robot
   */
  requestControl() {
    this.send({
      type: 'request_control'
    });
  }

  /**
   * Libera el control
   */
  releaseControl() {
    this.send({
      type: 'release_control'
    });
    this.hasControl = false;
  }

  /**
   * Envía un comando simple al robot
   */
  sendSimpleCommand(command) {
    if (!this.authenticated) {
      console.error('No autenticado');
      return false;
    }

    this.send({
      type: 'command',
      command
    });
    return true;
  }

  /**
   * Envía comando de velocidad
   */
  sendVelocity(linear, angular) {
    if (!this.authenticated) {
      console.error('No autenticado');
      return false;
    }

    this.send({
      type: 'command',
      command: 'move',
      data: {
        velocity: {
          linear: linear || { x: 0, y: 0, z: 0 },
          angular: angular || { x: 0, y: 0, z: 0 }
        }
      }
    });
    return true;
  }

  /**
   * Envía comando complejo
   */
  sendComplexCommand(command, value) {
    if (!this.authenticated) {
      console.error('No autenticado');
      return false;
    }

    this.send({
      type: 'command',
      command,
      data: { value }
    });
    return true;
  }

  /**
   * Detiene el robot
   */
  stop() {
    return this.sendVelocity(
      { x: 0, y: 0, z: 0 },
      { x: 0, y: 0, z: 0 }
    );
  }

  /**
   * Solicita estado del sistema
   */
  requestStatus() {
    this.send({ type: 'status' });
  }

  /**
   * Envía ping
   */
  ping() {
    this.send({ type: 'ping' });
  }

  /**
   * Envía mensaje al servidor
   */
  send(message) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    } else {
      console.error('WebSocket no está conectado');
    }
  }

  /**
   * Desconecta del servidor
   */
  disconnect() {
    if (this.ws) {
      this.ws.close();
    }
  }

  // Métodos para registrar callbacks
  onConnect(callback) { this.onConnectCallback = callback; }
  onDisconnect(callback) { this.onDisconnectCallback = callback; }
  onAuthSuccess(callback) { this.onAuthSuccessCallback = callback; }
  onTelemetry(callback) { this.onTelemetryCallback = callback; }
  onError(callback) { this.onErrorCallback = callback; }
}

// ============================================
// EJEMPLO DE USO
// ============================================

// Crear cliente
const client = new JueyingClient('ws://192.168.1.103:8080');

// Configurar callbacks
client.onConnect(() => {
  console.log('Conectado! Autenticando...');
  client.authenticate('operator1', 'password123');
});

client.onAuthSuccess((user) => {
  console.log('Autenticado como:', user.username);
  console.log('Rol:', user.role);
  console.log('Permisos:', user.permissions);
  
  // Solicitar control
  client.requestControl();
  
  // Después de 2 segundos, levantar el robot
  setTimeout(() => {
    console.log('Enviando comando: stand');
    client.sendSimpleCommand('stand');
  }, 2000);
});

client.onTelemetry((topic, data) => {
  // Procesar telemetría según el tópico
  switch(topic) {
    case 'odometry':
      const pos = data.pose.pose.position;
      console.log(`Posición: x=${pos.x.toFixed(2)}, y=${pos.y.toFixed(2)}, z=${pos.z.toFixed(2)}`);
      break;
      
    case 'imu':
      const orient = data.orientation;
      console.log(`Orientación: x=${orient.x.toFixed(2)}, y=${orient.y.toFixed(2)}, z=${orient.z.toFixed(2)}, w=${orient.w.toFixed(2)}`);
      break;
      
    case 'joints':
      console.log(`Juntas: ${data.name.length} articulaciones`);
      break;
      
    case 'handle':
      console.log(`Estado del robot: ${JSON.stringify(data)}`);
      break;
  }
});

client.onError((code, message) => {
  console.error(`Error [${code}]:`, message);
});

client.onDisconnect(() => {
  console.log('Conexión perdida. Intentando reconectar...');
});

// Conectar
client.connect();

// ============================================
// EJEMPLOS DE COMANDOS
// ============================================

// Comandos simples
// client.sendSimpleCommand('stand');     // Levantarse
// client.sendSimpleCommand('sit');       // Sentarse
// client.sendSimpleCommand('damping');   // Modo amortiguación
// client.sendSimpleCommand('walk');      // Caminar
// client.sendSimpleCommand('trot');      // Trotar

// Control de velocidad
// Avanzar a 0.5 m/s
// client.sendVelocity({ x: 0.5, y: 0, z: 0 }, { x: 0, y: 0, z: 0 });

// Girar en su lugar
// client.sendVelocity({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0.5 });

// Detener
// client.stop();

// Comandos complejos
// client.sendComplexCommand('body_height', 0.3);  // Altura 0.3m
// client.sendComplexCommand('pitch', 0.2);        // Inclinación
// client.sendComplexCommand('roll', 0.1);         // Inclinación lateral
// client.sendComplexCommand('yaw', 0.5);          // Rotación

// ============================================
// INTEGRACIÓN CON INTERFAZ HTML
// ============================================

/*
// En tu HTML:
<button onclick="robot.sendSimpleCommand('stand')">Levantarse</button>
<button onclick="robot.sendSimpleCommand('sit')">Sentarse</button>
<button onclick="robot.stop()">Detener</button>

// Joystick para movimiento:
function updateJoystick(x, y) {
  robot.sendVelocity(
    { x: y, y: 0, z: 0 },      // Forward/backward
    { x: 0, y: 0, z: -x }      // Left/right rotation
  );
}

// Mostrar telemetría:
robot.onTelemetry((topic, data) => {
  if (topic === 'odometry') {
    document.getElementById('position').textContent = 
      `X: ${data.pose.pose.position.x.toFixed(2)} Y: ${data.pose.pose.position.y.toFixed(2)}`;
  }
});
*/
