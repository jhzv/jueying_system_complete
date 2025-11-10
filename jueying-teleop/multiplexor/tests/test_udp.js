/**
 * Script de Prueba UDP
 * Prueba comunicación directa con el Motion Host del Jueying Lite3
 * 
 * Uso:
 *   node tests/test_udp.js
 *   node tests/test_udp.js --host 192.168.1.120 --port 43893
 */

const dgram = require('dgram');

// Configuración
const config = {
  host: process.argv.includes('--host') 
    ? process.argv[process.argv.indexOf('--host') + 1] 
    : '192.168.1.120',
  port: process.argv.includes('--port')
    ? parseInt(process.argv[process.argv.indexOf('--port') + 1])
    : 43893
};

// Comandos de prueba
const COMMANDS = {
  HEARTBEAT: 0x21040001,
  MODE_POSE: 0x21010D05,
  MODE_MOVE: 0x21010D06,
  ACTION_SIT_STAND: 0x21010202
};

console.log('='.repeat(60));
console.log('Test de Comunicación UDP - Jueying Lite3');
console.log('='.repeat(60));
console.log(`Target: ${config.host}:${config.port}`);
console.log('');

// Crear socket UDP
const socket = dgram.createSocket('udp4');

socket.on('error', (err) => {
  console.error('❌ Socket error:', err.message);
  socket.close();
  process.exit(1);
});

socket.on('listening', () => {
  const address = socket.address();
  console.log(`✅ Socket listening on ${address.address}:${address.port}`);
  console.log('');
  
  runTests();
});

/**
 * Enviar comando UDP
 */
function sendCommand(name, code, value = 0, type = 0) {
  return new Promise((resolve) => {
    const buffer = Buffer.alloc(12);
    buffer.writeInt32LE(code, 0);
    buffer.writeInt32LE(value, 4);
    buffer.writeInt32LE(type, 8);
    
    console.log(`📤 Enviando: ${name}`);
    console.log(`   Code: 0x${code.toString(16)}`);
    console.log(`   Value: ${value}`);
    console.log(`   Type: ${type}`);
    
    socket.send(buffer, config.port, config.host, (err) => {
      if (err) {
        console.log(`   ❌ Error: ${err.message}`);
        resolve(false);
      } else {
        console.log(`   ✅ Enviado (${buffer.length} bytes)`);
        resolve(true);
      }
      console.log('');
    });
  });
}

/**
 * Ejecutar batería de pruebas
 */
async function runTests() {
  console.log('Iniciando pruebas...');
  console.log('');
  
  try {
    // Test 1: Heartbeat
    console.log('Test 1: HEARTBEAT');
    console.log('-'.repeat(40));
    await sendCommand('Heartbeat', COMMANDS.HEARTBEAT);
    await sleep(500);
    
    // Test 2: Cambiar a modo POSE
    console.log('Test 2: MODE POSE');
    console.log('-'.repeat(40));
    await sendCommand('Mode POSE', COMMANDS.MODE_POSE);
    await sleep(1000);
    
    // Test 3: Heartbeat de nuevo
    console.log('Test 3: HEARTBEAT (verificación)');
    console.log('-'.repeat(40));
    await sendCommand('Heartbeat', COMMANDS.HEARTBEAT);
    await sleep(500);
    
    // Test 4: Cambiar a modo MOVE
    console.log('Test 4: MODE MOVE');
    console.log('-'.repeat(40));
    await sendCommand('Mode MOVE', COMMANDS.MODE_MOVE);
    await sleep(1000);
    
    // Test 5: Heartbeat final
    console.log('Test 5: HEARTBEAT (final)');
    console.log('-'.repeat(40));
    await sendCommand('Heartbeat', COMMANDS.HEARTBEAT);
    
    console.log('='.repeat(60));
    console.log('✅ Pruebas completadas');
    console.log('='.repeat(60));
    console.log('');
    console.log('Resultados:');
    console.log('- Si todos los comandos se enviaron correctamente, la');
    console.log('  comunicación UDP con el Motion Host está funcionando.');
    console.log('');
    console.log('- Observa el robot para verificar si responde a los comandos.');
    console.log('');
    console.log('Notas:');
    console.log('- UDP es un protocolo sin confirmación (fire-and-forget)');
    console.log('- No recibiremos respuestas del robot');
    console.log('- Verificar comportamiento físico del robot');
    console.log('');
    
  } catch (error) {
    console.error('❌ Error durante pruebas:', error);
  } finally {
    socket.close();
    console.log('Socket cerrado');
  }
}

/**
 * Helper para pausas
 */
function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

// Iniciar
socket.bind();
