# Multiplexor Jueying Lite3 v2.1 - Arquitectura Híbrida ROS + UDP

Multiplexor WebSocket para control remoto del robot cuadrúpedo Jueying Lite3 usando protocolo híbrido.

## 🔧 Arquitectura Híbrida

### ❌ Problema Identificado
El robot Jueying Lite3 **NO tiene implementados** los tópicos `/simple_cmd`, `/complex_cmd` o `/pose_cmd` en ROS.

**Hallazgos:**
- ✅ `/cmd_vel` existe y funciona (solo movimiento continuo)
- ❌ `/simple_cmd` NO existe
- ❌ `/complex_cmd` NO existe  
- ❌ `/pose_cmd` NO existe

### ✅ Solución: Protocolo Híbrido

```
┌─────────────────────────────────────────────────────────────┐
│                    MULTIPLEXOR HÍBRIDO                       │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ROS/Rosbridge (ws://localhost:9090)                        │
│  ├── SOLO Telemetría (lectura):                             │
│  │   ├── /leg_odom → Odometría                              │
│  │   ├── /imu/data → IMU                                    │
│  │   ├── /joint_states → Estado juntas                      │
│  │   └── /handle_state → Estado robot                       │
│  │                                                           │
│  └── Movimiento continuo (opcional):                        │
│      └── /cmd_vel → ros2qnx → UDP Motion Host               │
│                                                              │
│  UDP Directo (192.168.1.120:43893)                          │
│  └── TODOS los comandos de control:                         │
│      ├── Modos: POSE (0x21010D05) / MOVE (0x21010D06)       │
│      ├── Poses: sit_stand, say_hello, etc.                  │
│      ├── Movimiento: ejes FB/LR/Rotation                    │
│      ├── Gaits: flat_slow, flat_fast, etc.                  │
│      └── Heartbeat: 0x21040001 (4Hz)                        │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

## 📋 Características

- ✅ **Protocolo UDP Nativo**: Control directo al Motion Host (como lite3-local-web-control)
- ✅ **Telemetría ROS**: Feedback en tiempo real vía rosbridge
- ✅ **Autenticación JWT**: Control de acceso seguro
- ✅ **Gestión de Sesiones**: Sistema de prioridades y control exclusivo
- ✅ **Rate Limiting**: Protección contra comandos excesivos (20Hz para movimiento, 10Hz para comandos)
- ✅ **Heartbeat Automático**: Keep-alive a 4Hz
- ✅ **Logging Completo**: Winston con rotación de archivos
- ✅ **Reconexión Automática**: A rosbridge y UDP

## 🏗️ Flujo de Datos

```
┌──────────────────┐
│  Frontend Web    │
│  (puerto 3000)   │
└────────┬─────────┘
         │ WebSocket (wss://robot:8080)
         ↓
┌──────────────────────────────────────────────┐
│        Multiplexor v2.1 (puerto 8080)        │
│  ┌──────────────────┐  ┌─────────────────┐  │
│  │   ROS Client     │  │   UDP Client    │  │
│  │  (rosbridge)     │  │  (Motion Host)  │  │
│  └────────┬─────────┘  └────────┬────────┘  │
└───────────┼──────────────────────┼───────────┘
            │                      │
            │ WebSocket            │ UDP
            │ (9090)               │ (43893)
            ↓                      ↓
  ┌─────────────────┐    ┌──────────────────┐
  │  Rosbridge      │    │   Motion Host    │
  │  (Jetson)       │    │   (RK3588)       │
  └────────┬────────┘    │  192.168.1.120   │
           │             └──────────────────┘
           │ ROS Topics           │
           ↓                      ↓
  ┌─────────────────┐    ┌──────────────────┐
  │   qnx2ros       │◄───│  Robot Hardware  │
  │  (telemetría)   │    │  (sensores, etc) │
  └─────────────────┘    └──────────────────┘
```

## 📦 Instalación

### Requisitos Previos
```bash
# Verificar Node.js (>= 18)
node --version

# Verificar ROS activo
rosnode list

# Verificar rosbridge
netstat -tlnp | grep 9090
```

### Instalación

```bash
# 1. Backup del multiplexor anterior
cd /home/ysc/jueying-teleop/
mv multiplexor multiplexor-backup-$(date +%Y%m%d)

# 2. Copiar nuevo multiplexor
cp -r /ruta/multiplexor-v2.1-hybrid multiplexor
cd multiplexor

# 3. Instalar dependencias
npm install

# 4. Configurar entorno
cp .env.example .env
nano .env
```

### Configuración `.env`

```bash
# Servidor
PORT=8080
LOG_LEVEL=info

# ROS
ROSBRIDGE_URL=ws://localhost:9090

# Robot (Motion Host)
ROBOT_MOTION_HOST=192.168.1.120
ROBOT_MOTION_PORT=43893
ROBOT_PERCEPTION_HOST=192.168.1.103

# Seguridad
JWT_SECRET=CAMBIAR_EN_PRODUCCION_secreto_ultra_seguro_2024

# Control
HEARTBEAT_INTERVAL=250
MOVEMENT_TIMEOUT=250
COMMAND_TIMEOUT=1000
MAX_CLIENTS=5

# Rate Limiting
RATE_LIMIT_MOVEMENT=20
RATE_LIMIT_COMMANDS=10
```

## 🚀 Uso

### Inicio Manual
```bash
cd /home/ysc/jueying-teleop/multiplexor
npm start
```

### Verificar Estado
```bash
# Ver logs
tail -f logs/multiplexor.log

# Verificar proceso
ps aux | grep "node.*server.js"

# Verificar puerto
netstat -tlnp | grep 8080
```

### Inicio Automático (systemd)
```bash
# Copiar servicio
sudo cp systemd/jueying-multiplexor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable jueying-multiplexor
sudo systemctl start jueying-multiplexor

# Verificar estado
sudo systemctl status jueying-multiplexor
```

## 📡 Protocolo WebSocket

### 1. Conexión y Autenticación

```javascript
const ws = new WebSocket('ws://192.168.1.103:8080');

// Autenticar
ws.send(JSON.stringify({
  type: 'auth',
  username: 'operator1',
  password: 'password123'
}));

// Respuesta exitosa
{
  type: 'auth_success',
  token: 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...',
  user: {
    username: 'operator1',
    role: 'operator',
    permissions: ['control', 'view']
  }
}
```

### 2. Comandos de Control

#### Cambiar Modo (POSE / MOVE)
```javascript
// Modo POSE (para poses/acciones)
ws.send(JSON.stringify({
  type: 'command',
  command: 'set_mode',
  data: { mode: 'pose' }
}));

// Modo MOVE (para movimiento)
ws.send(JSON.stringify({
  type: 'command',
  command: 'set_mode',
  data: { mode: 'move' }
}));
```

#### Acciones (solo en modo POSE)
```javascript
// Sentarse/Levantarse
ws.send(JSON.stringify({
  type: 'command',
  command: 'sit_stand'
}));

// Saludar
ws.send(JSON.stringify({
  type: 'command',
  command: 'say_hello'
}));

// Otras acciones disponibles:
// - long_jump
// - twist_jump
// - moonwalk
// - twist
// - zero
```

#### Movimiento (solo en modo MOVE)
```javascript
// Movimiento con joystick (valores -100 a 100)
ws.send(JSON.stringify({
  type: 'command',
  command: 'move',
  data: {
    forward_back: 50,    // -100 (atrás) a 100 (adelante)
    left_right: 0,       // -100 (izq) a 100 (der)
    rotation: 20         // -100 (izq) a 100 (der)
  }
}));

// Detener
ws.send(JSON.stringify({
  type: 'command',
  command: 'stop'
}));
```

#### Cambiar Gait (solo en modo MOVE)
```javascript
ws.send(JSON.stringify({
  type: 'command',
  command: 'set_gait',
  data: { gait: 'flat_fast' }
}));

// Gaits disponibles:
// - flat_slow
// - flat_medium
// - flat_fast
// - flat_crawl
// - rug_grip
// - rug_general
// - rug_hstep
```

### 3. Recibir Telemetría

```javascript
ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);
  
  switch(msg.type) {
    case 'telemetry':
      // Datos de sensores ROS
      switch(msg.topic) {
        case 'odometry':
          console.log('Posición:', msg.data.pose);
          console.log('Velocidad:', msg.data.twist);
          break;
        case 'imu':
          console.log('Orientación:', msg.data.orientation);
          console.log('Aceleración:', msg.data.linear_acceleration);
          break;
        case 'joints':
          console.log('Posiciones juntas:', msg.data.position);
          break;
        case 'handle':
          console.log('Estado robot:', msg.data);
          break;
      }
      break;
      
    case 'status':
      console.log('Estado del sistema:', msg.data);
      break;
      
    case 'error':
      console.error('Error:', msg.message);
      break;
  }
};
```

## 🎮 Comandos UDP Disponibles

### Modos
- `MODE_POSE` (0x21010D05) - Poses y acciones
- `MODE_MOVE` (0x21010D06) - Movimiento continuo

### Acciones (Modo POSE)
- `ACTION_SIT_STAND` (0x21010202) - Sentarse/Levantarse
- `ACTION_SAY_HELLO` (0x21010507) - Saludar
- `ACTION_LONG_JUMP` (0x21010508) - Salto largo
- `ACTION_TWIST_JUMP` (0x21010502) - Salto con giro
- `ACTION_MOONWALK` (0x21010506) - Moonwalk
- `ACTION_TWIST` (0x21010500) - Giro
- `ACTION_ZERO` (0x21010501) - Posición cero

### Movimiento (Modo MOVE)
- `AXIS_TRANSLATION_FB` (0x21010130) - Adelante/Atrás
- `AXIS_TRANSLATION_LR` (0x21010131) - Izquierda/Derecha
- `AXIS_ROTATION` (0x21010135) - Rotación

### Gaits (Modo MOVE)
- `GAIT_FLAT_SLOW` (0x21010300)
- `GAIT_FLAT_MEDIUM` (0x21010307)
- `GAIT_FLAT_FAST` (0x21010303)
- `GAIT_FLAT_CRAWL` (0x21010308)
- `GAIT_RUG_GRIP` (0x21010304)
- `GAIT_RUG_GENERAL` (0x21010305)
- `GAIT_RUG_HSTEP` (0x21010306)

### Sistema
- `HEARTBEAT` (0x21040001) - Keep alive (automático a 4Hz)
- `EMERGENCY_STOP` (0x21010A00) - Paro de emergencia
- `SAVE_DATA` (0x21030001) - Guardar datos

## 🛠️ Depuración

### Ver Comunicación UDP
```bash
# Monitorear tráfico UDP al Motion Host
sudo tcpdump -i any -n udp port 43893 -X

# Ver logs del multiplexor
tail -f logs/multiplexor.log | grep UDP
```

### Verificar Telemetría ROS
```bash
# Ver tópicos publicados
rostopic list

# Monitorear odometría
rostopic echo /leg_odom

# Verificar frecuencia
rostopic hz /leg_odom
```

### Probar Comandos UDP Directos
```bash
# Instalar herramienta de prueba
cd /home/ysc/jueying-teleop/multiplexor
npm run test:udp

# O usar Python
cd tests/
python3 test_udp_commands.py --host 192.168.1.120 --command stand
```

## 🔒 Seguridad

### Usuarios por Defecto
| Usuario | Password | Rol | Permisos |
|---------|----------|-----|----------|
| admin | admin123 | admin | control, view, configure |
| operator1 | password123 | operator | control, view |

⚠️ **CAMBIAR CONTRASEÑAS EN PRODUCCIÓN**

### Agregar Usuario
```bash
# Generar hash de password
npm run hash:password -- "nueva_password"

# Editar config/users.js y agregar:
{
  username: 'nuevo_usuario',
  passwordHash: '$2b$10$...',
  role: 'operator',
  permissions: ['control', 'view']
}
```

### Configurar HTTPS/WSS
```bash
# Generar certificados
openssl req -x509 -newkey rsa:4096 -nodes \
  -keyout certs/key.pem -out certs/cert.pem -days 365

# Actualizar .env
USE_SSL=true
SSL_KEY=certs/key.pem
SSL_CERT=certs/cert.pem
```

## 📊 Monitoreo

### Dashboard de Estado
```javascript
// Solicitar estado del sistema
ws.send(JSON.stringify({ type: 'status' }));

// Respuesta
{
  type: 'status',
  data: {
    server: {
      uptime: 123456,
      clients: 2,
      memory: { heapUsed: 45.2, heapTotal: 128 }
    },
    ros: {
      connected: true,
      subscribers: ['/leg_odom', '/imu/data', ...],
      lastMessage: '2024-10-16T12:34:56Z'
    },
    udp: {
      connected: true,
      mode: 'move',
      gait: 'flat_fast',
      heartbeatRunning: true,
      lastCommand: 'move',
      lastCommandTime: '2024-10-16T12:34:55Z'
    },
    control: {
      currentController: { username: 'operator1', ... },
      activeClients: 2,
      queuedRequests: 0
    }
  }
}
```

## 🐛 Solución de Problemas

### Multiplexor no conecta a ROS
```bash
# Verificar rosbridge
rosnode list | grep rosbridge
netstat -tlnp | grep 9090

# Reiniciar rosbridge
rosnode kill /rosbridge_websocket
roslaunch rosbridge_server rosbridge_websocket.launch
```

### Comandos UDP no llegan al robot
```bash
# Verificar conectividad
ping 192.168.1.120

# Verificar puerto UDP
nc -u -v 192.168.1.120 43893

# Ver logs de errores
tail -f logs/multiplexor.log | grep "UDP error"
```

### No recibo telemetría
```bash
# Verificar nodos ROS
rosnode list | grep qnx2ros

# Verificar publicación
rostopic hz /leg_odom
rostopic hz /imu/data

# Ver logs de suscripciones
tail -f logs/multiplexor.log | grep "ROS subscriber"
```

## 📚 Estructura del Proyecto

```
multiplexor-v2.1-hybrid/
├── src/
│   ├── server.js              # Servidor principal
│   ├── auth/                  # Autenticación JWT
│   │   ├── jwt.js
│   │   └── middleware.js
│   ├── control/               # Gestor de control
│   │   └── manager.js
│   ├── ros/                   # Cliente ROS
│   │   ├── client.js
│   │   └── validator.js
│   ├── udp/                   # Cliente UDP ⭐ NUEVO
│   │   ├── client.js          # Protocolo UDP nativo
│   │   └── constants.js       # Códigos de comandos
│   └── utils/                 # Utilidades
│       ├── logger.js
│       └── rateLimiter.js
├── config/
│   └── users.js               # Configuración de usuarios
├── tests/
│   ├── test_udp_commands.py   # Pruebas UDP
│   └── test_websocket.html    # Pruebas WebSocket
├── systemd/
│   └── jueying-multiplexor.service
├── .env.example
├── package.json
└── README.md
```

## 📄 Licencia

Propiedad de Skaler S.A. - Uso interno solamente.

---

**Versión**: 2.1.0 (Híbrido ROS + UDP)  
**Fecha**: Octubre 2024  
**Autor**: Jhosep Amaury Zapata Varela  
**Estado**: ✅ En desarrollo
