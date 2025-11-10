#!/usr/bin/env node
// test/integration-test.js
// Test de integración del multiplexor

const WebSocket = require('ws');

const MULTIPLEXOR_URL = process.env.MULTIPLEXOR_URL || 'ws://localhost:8080';
const TEST_USER = process.env.TEST_USER || 'operator1';
const TEST_PASS = process.env.TEST_PASS || 'password123';

console.log('====================================');
console.log('Test de Integración - Multiplexor');
console.log('====================================\n');

let ws;
let authenticated = false;
let testsCompleted = 0;
let testsFailed = 0;

// Tests a ejecutar
const tests = [
  { name: 'Conexión al multiplexor', fn: testConnection },
  { name: 'Autenticación', fn: testAuth },
  { name: 'Solicitud de control', fn: testRequestControl },
  { name: 'Envío de comando simple', fn: testSimpleCommand },
  { name: 'Envío de velocidad', fn: testVelocityCommand },
  { name: 'Solicitud de estado', fn: testStatus },
  { name: 'Ping/Pong', fn: testPing }
];

function testConnection() {
  return new Promise((resolve, reject) => {
    console.log('→ Conectando a', MULTIPLEXOR_URL);
    
    ws = new WebSocket(MULTIPLEXOR_URL);
    
    ws.on('open', () => {
      console.log('✓ Conexión establecida');
      resolve();
    });

    ws.on('error', (error) => {
      console.error('✗ Error de conexión:', error.message);
      reject(error);
    });

    ws.on('message', handleMessage);

    setTimeout(() => {
      reject(new Error('Timeout en conexión'));
    }, 5000);
  });
}

function testAuth() {
  return new Promise((resolve, reject) => {
    console.log(`→ Autenticando como ${TEST_USER}`);
    
    const authHandler = (message) => {
      if (message.type === 'auth_success') {
        authenticated = true;
        console.log('✓ Autenticación exitosa');
        console.log('  Usuario:', message.user.username);
        console.log('  Rol:', message.user.role);
        console.log('  Permisos:', message.user.permissions.join(', '));
        ws.removeListener('message', authHandler);
        resolve();
      } else if (message.type === 'error' && message.error.code === 'auth_failed') {
        console.error('✗ Autenticación fallida');
        reject(new Error('Auth failed'));
      }
    };

    ws.on('message', (data) => {
      const message = JSON.parse(data);
      authHandler(message);
    });

    ws.send(JSON.stringify({
      type: 'auth',
      username: TEST_USER,
      password: TEST_PASS
    }));

    setTimeout(() => {
      reject(new Error('Timeout en autenticación'));
    }, 3000);
  });
}

function testRequestControl() {
  return new Promise((resolve, reject) => {
    if (!authenticated) {
      reject(new Error('No autenticado'));
      return;
    }

    console.log('→ Solicitando control');

    const controlHandler = (message) => {
      if (message.type === 'control_granted') {
        console.log('✓ Control concedido');
        ws.removeListener('message', controlHandler);
        resolve();
      } else if (message.type === 'control_denied') {
        console.log('⚠ Control denegado:', message.error.message);
        resolve(); // No es un error crítico para el test
      }
    };

    ws.on('message', (data) => {
      const message = JSON.parse(data);
      controlHandler(message);
    });

    ws.send(JSON.stringify({
      type: 'request_control'
    }));

    setTimeout(() => {
      reject(new Error('Timeout en solicitud de control'));
    }, 3000);
  });
}

function testSimpleCommand() {
  return new Promise((resolve, reject) => {
    console.log('→ Enviando comando simple (stand)');

    const commandHandler = (message) => {
      if (message.type === 'command_ack' && message.command === 'stand') {
        console.log('✓ Comando confirmado');
        ws.removeListener('message', commandHandler);
        resolve();
      } else if (message.type === 'error') {
        console.error('✗ Error en comando:', message.error.message);
        reject(new Error(message.error.message));
      }
    };

    ws.on('message', (data) => {
      const message = JSON.parse(data);
      commandHandler(message);
    });

    ws.send(JSON.stringify({
      type: 'command',
      command: 'stand'
    }));

    setTimeout(() => {
      reject(new Error('Timeout en comando simple'));
    }, 3000);
  });
}

function testVelocityCommand() {
  return new Promise((resolve, reject) => {
    console.log('→ Enviando comando de velocidad');

    const commandHandler = (message) => {
      if (message.type === 'command_ack' && message.command === 'move') {
        console.log('✓ Comando de velocidad confirmado');
        ws.removeListener('message', commandHandler);
        resolve();
      } else if (message.type === 'error') {
        console.error('✗ Error en comando:', message.error.message);
        reject(new Error(message.error.message));
      }
    };

    ws.on('message', (data) => {
      const message = JSON.parse(data);
      commandHandler(message);
    });

    ws.send(JSON.stringify({
      type: 'command',
      command: 'move',
      data: {
        velocity: {
          linear: { x: 0.5, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 }
        }
      }
    }));

    setTimeout(() => {
      reject(new Error('Timeout en comando de velocidad'));
    }, 3000);
  });
}

function testStatus() {
  return new Promise((resolve, reject) => {
    console.log('→ Solicitando estado del sistema');

    const statusHandler = (message) => {
      if (message.type === 'status') {
        console.log('✓ Estado recibido');
        console.log('  ROS conectado:', message.ros.connected);
        console.log('  Clientes activos:', message.server.clients);
        console.log('  Publishers:', message.ros.publishers.length);
        console.log('  Subscribers:', message.ros.subscribers.length);
        ws.removeListener('message', statusHandler);
        resolve();
      }
    };

    ws.on('message', (data) => {
      const message = JSON.parse(data);
      statusHandler(message);
    });

    ws.send(JSON.stringify({
      type: 'status'
    }));

    setTimeout(() => {
      reject(new Error('Timeout en solicitud de estado'));
    }, 3000);
  });
}

function testPing() {
  return new Promise((resolve, reject) => {
    console.log('→ Enviando ping');

    const pingHandler = (message) => {
      if (message.type === 'pong') {
        console.log('✓ Pong recibido');
        ws.removeListener('message', pingHandler);
        resolve();
      }
    };

    ws.on('message', (data) => {
      const message = JSON.parse(data);
      pingHandler(message);
    });

    ws.send(JSON.stringify({
      type: 'ping'
    }));

    setTimeout(() => {
      reject(new Error('Timeout en ping'));
    }, 3000);
  });
}

function handleMessage(data) {
  try {
    const message = JSON.parse(data);
    
    if (message.type === 'welcome') {
      console.log('  Mensaje de bienvenida:', message.message);
    } else if (message.type === 'telemetry') {
      console.log(`  [Telemetría] ${message.topic}`);
    }
  } catch (error) {
    // Ignorar errores de parsing en handler general
  }
}

// Ejecutar tests secuencialmente
async function runTests() {
  console.log(`Ejecutando ${tests.length} tests...\n`);

  for (const test of tests) {
    try {
      console.log(`\n[${testsCompleted + 1}/${tests.length}] ${test.name}`);
      await test.fn();
      testsCompleted++;
    } catch (error) {
      console.error(`✗ Test fallido: ${error.message}`);
      testsFailed++;
    }
    
    // Pequeña pausa entre tests
    await new Promise(resolve => setTimeout(resolve, 500));
  }

  // Cerrar conexión
  if (ws) {
    ws.close();
  }

  // Resumen
  console.log('\n====================================');
  console.log('Resumen de Tests');
  console.log('====================================');
  console.log(`Total: ${tests.length}`);
  console.log(`✓ Exitosos: ${testsCompleted}`);
  console.log(`✗ Fallidos: ${testsFailed}`);
  
  if (testsFailed === 0) {
    console.log('\n🎉 ¡Todos los tests pasaron!');
    process.exit(0);
  } else {
    console.log('\n⚠ Algunos tests fallaron');
    process.exit(1);
  }
}

// Manejo de señales
process.on('SIGINT', () => {
  console.log('\n\nTest interrumpido por usuario');
  if (ws) ws.close();
  process.exit(130);
});

// Iniciar tests
runTests().catch(error => {
  console.error('\n✗ Error fatal:', error.message);
  if (ws) ws.close();
  process.exit(1);
});
