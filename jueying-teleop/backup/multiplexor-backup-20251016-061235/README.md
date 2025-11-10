# Multiplexor Jueying Lite3 - Versión Corregida

Multiplexor WebSocket para control remoto del robot cuadrúpedo Jueying Lite3 a través de ROS.

## 🔧 Correcciones Principales

### ✅ Problema Resuelto
**Antes:** El multiplexor intentaba publicar a `/pose_cmd` que NO existe en el robot.
```
Error: Cannot infer topic type for topic /pose_cmd
```

**Ahora:** Usa los tópicos ROS correctos verificados del Jueying Lite3:
- ✅ `/cmd_vel` - Control de velocidades
- ✅ `/simple_cmd` - Comandos discretos (stand, sit, etc)
- ✅ `/complex_cmd` - Comandos con parámetros
- ✅ `/leg_odom`, `/imu/data`, `/joint_states`, `/handle_state` - Feedback del robot

## 📋 Características

- ✅ **Tópicos ROS Correctos**: Comunicación con nodos `qnx2ros` y `ros2qnx`
- ✅ **Autenticación JWT**: Control de acceso seguro
- ✅ **Gestión de Sesiones**: Sistema de prioridades y control exclusivo
- ✅ **Telemetría en Tiempo Real**: Broadcast de odometría, IMU, juntas y estado
- ✅ **Rate Limiting**: Protección contra comandos excesivos
- ✅ **Logging Completo**: Winston con rotación de archivos
- ✅ **Reconexión Automática**: A rosbridge en caso de caída

## 🏗️ Arquitectura

```
Frontend Web (puerto 3000)
    ↓ WebSocket
Multiplexor (puerto 8080) [ESTE COMPONENTE]
    ↓ WebSocket  
Rosbridge (puerto 9090)
    ↓ ROS Topics
ros2qnx → UDP → Motion Host (RK3588) → Robot
    ↑
qnx2ros ← UDP ← Motion Host ← Sensores
    ↓ ROS Topics
Rosbridge → Multiplexor → Frontend
```

## 📦 Instalación

### Requisitos Previos
- Node.js 18+ (ya instalado vía NVM en Jetson Xavier NX)
- ROS Melodic/Noetic
- rosbridge_server corriendo en puerto 9090
- Nodos `qnx2ros` y `ros2qnx` del robot activos

### Pasos de Instalación

1. **Copiar archivos al Jetson**
```bash
# En el Jetson Xavier NX
cd /home/ysc/jueying-teleop/
mv multiplexor multiplexor-old  # Backup del anterior
cp -r /ruta/multiplexor-actualizado multiplexor
cd multiplexor
```

2. **Instalar dependencias**
```bash
npm install
```

3. **Configurar variables de entorno**
```bash
cp .env.example .env
nano .env
```

Editar `.env`:
```bash
PORT=8080
ROSBRIDGE_URL=ws://localhost:9090
JWT_SECRET=tu-secreto-super-seguro-cambiar-en-produccion
ROBOT_MOTION_HOST=192.168.1.120
ROBOT_PERCEPTION_HOST=192.168.1.103
LOG_LEVEL=info
```

4. **Verificar usuarios** (opcional)
```bash
nano config/users.js
# Agregar/modificar usuarios según necesidad
```

## 🚀 Uso

### Inicio Manual
```bash
cd /home/ysc/jueying-teleop/multiplexor
npm start
```

### Inicio con el Script del Sistema
```bash
~/start-jueying-native.sh
```

El script de inicio debería incluir:
```bash
#!/bin/bash
# Iniciar ROS Master
roscore &
sleep 2

# Iniciar rosbridge
roslaunch rosbridge_server rosbridge_websocket.launch &
sleep 3

# Iniciar multiplexor
cd /home/ysc/jueying-teleop/multiplexor
npm start &

# Iniciar frontend (si está separado)
# cd /home/ysc/jueying-teleop/frontend
# python3 -m http.server 3000 &
```

### Verificar Estado
```bash
~/status-jueying-teleop.sh
```

Debería mostrar:
```
✅ ROS Master: Activo
✅ Rosbridge: Puerto 9090 activo
✅ Multiplexor: Puerto 8080 activo
✅ Frontend: Puerto 3000 activo
```

## 📡 Protocolo WebSocket

### Conexión
```javascript
const ws = new WebSocket('ws://192.168.1.103:8080');
```

### Autenticación
```javascript
ws.send(JSON.stringify({
  type: 'auth',
  username: 'operator1',
  password: 'password123'
}));

// Respuesta:
{
  type: 'auth_success',
  token: 'jwt_token...',
  user: {
    username: 'operator1',
    role: 'operator',
    permissions: ['control', 'view']
  }
}
```

### Enviar Comandos

**Comando Simple (stand, sit, etc):**
```javascript
ws.send(JSON.stringify({
  type: 'command',
  command: 'stand'
}));
```

**Control de Velocidad:**
```javascript
ws.send(JSON.stringify({
  type: 'command',
  command: 'move',
  data: {
    velocity: {
      linear: { x: 0.5, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0.2 }
    }
  }
}));
```

**Comando Complejo (altura, pitch, roll):**
```javascript
ws.send(JSON.stringify({
  type: 'command',
  command: 'body_height',
  data: {
    value: 0.3  // metros
  }
}));
```

### Recibir Telemetría
```javascript
ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  
  if (message.type === 'telemetry') {
    switch(message.topic) {
      case 'odometry':
        // message.data contiene nav_msgs/Odometry
        updatePosition(message.data);
        break;
      case 'imu':
        // message.data contiene sensor_msgs/Imu
        updateOrientation(message.data);
        break;
      case 'joints':
        // message.data contiene sensor_msgs/JointState
        updateJoints(message.data);
        break;
      case 'handle':
        // message.data contiene HandleState
        updateRobotState(message.data);
        break;
    }
  }
};
```

## 🎮 Comandos Disponibles

### Comandos Simples (simple_cmd)
- `stand` - Levantarse
- `sit` - Sentarse
- `damping` - Modo amortiguación
- `walk` - Caminar
- `trot` - Trotar
- `run` - Correr

### Comandos de Velocidad (cmd_vel)
- `move` - Control continuo de velocidad
- `stop` - Detener movimiento

### Comandos Complejos (complex_cmd)
- `body_height` - Altura del cuerpo
- `pitch` - Inclinación adelante/atrás
- `roll` - Inclinación lateral
- `yaw` - Rotación

## 🔍 Depuración

### Ver Logs
```bash
tail -f /tmp/multiplexor.log
```

### Verificar Tópicos ROS
```bash
rostopic list
# Debe mostrar:
# /cmd_vel
# /simple_cmd
# /complex_cmd
# /leg_odom
# /imu/data
# /joint_states
# /handle_state
```

### Probar Publicación Manual
```bash
# Comando simple
rostopic pub /simple_cmd message_transformer/SimpleCMD "data: 'stand'"

# Velocidad
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### Verificar rosbridge
```bash
# Desde otro terminal
wscat -c ws://localhost:9090
# Debería conectar sin errores
```

## 🛠️ Solución de Problemas

### Multiplexor no conecta a rosbridge
```bash
# Verificar que rosbridge está corriendo
ps aux | grep rosbridge
netstat -tlnp | grep 9090

# Reiniciar rosbridge si es necesario
rosnode kill /rosbridge_websocket
roslaunch rosbridge_server rosbridge_websocket.launch
```

### Comandos no llegan al robot
```bash
# Verificar que ros2qnx está corriendo
rosnode list | grep ros2qnx

# Ver si ros2qnx está recibiendo comandos
rostopic echo /cmd_vel
rostopic echo /simple_cmd

# Verificar conectividad UDP al Motion Host
ping 192.168.1.120
```

### No recibo telemetría
```bash
# Verificar que qnx2ros está corriendo
rosnode list | grep qnx2ros

# Ver si los topics están publicando
rostopic hz /leg_odom
rostopic hz /imu/data
rostopic hz /joint_states
```

## 📝 Usuarios por Defecto

| Usuario | Password | Rol | Permisos |
|---------|----------|-----|----------|
| admin | admin123 | admin | control, view, configure |
| operator1 | password123 | operator | control, view |

**⚠️ IMPORTANTE:** Cambiar las contraseñas en producción.

### Agregar Nuevo Usuario
```javascript
// En config/users.js
const bcrypt = require('bcrypt');

// Generar hash
bcrypt.hash('nueva_password', 10, (err, hash) => {
  console.log(hash);
});

// Agregar a users:
nuevoUsuario: {
  username: 'nuevo',
  passwordHash: '$2b$10$...hash...',
  role: 'operator',
  permissions: ['control', 'view']
}
```

## 🔐 Seguridad

- ✅ JWT con expiración configurable
- ✅ Passwords hasheados con bcrypt
- ✅ Rate limiting por cliente
- ✅ Sistema de permisos por roles
- ✅ Control de sesiones con timeouts
- ⚠️ Usar HTTPS/WSS en producción
- ⚠️ Cambiar JWT_SECRET en producción

## 📊 Monitoreo

### Estado del Sistema
```javascript
ws.send(JSON.stringify({ type: 'status' }));

// Respuesta incluye:
{
  type: 'status',
  ros: {
    connected: true,
    publishers: [...],
    subscribers: [...]
  },
  control: {
    currentController: {...},
    activeClients: [...]
  },
  server: {
    uptime: 12345,
    clients: 2,
    memory: {...}
  }
}
```

## 🔄 Actualización desde Versión Anterior

```bash
cd /home/ysc/jueying-teleop/

# Backup
mv multiplexor multiplexor-backup-$(date +%Y%m%d)

# Instalar nueva versión
cp -r /ruta/multiplexor-actualizado multiplexor
cd multiplexor
npm install

# Copiar configuración si existe
cp ../multiplexor-backup-*/. env . || cp .env.example .env

# Reiniciar
~/stop-jueying-teleop.sh
~/start-jueying-native.sh
```

## 📚 Referencias

- **Documentación ROS**: http://wiki.ros.org
- **rosbridge_suite**: https://github.com/RobotWebTools/rosbridge_suite
- **roslibjs**: https://github.com/RobotWebTools/roslibjs
- **Jueying SDK**: (documentación interna del fabricante)

## 🤝 Contribuir

Para reportar bugs o sugerir mejoras, contactar al equipo de desarrollo.

## 📄 Licencia

Propiedad de [Tu Organización] - Uso interno solamente.

---

**Versión**: 2.0.0  
**Fecha**: Octubre 2025  
**Estado**: ✅ Producción (Tópicos ROS Corregidos)
