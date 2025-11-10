# Guía Rápida de Inicio - Multiplexor Jueying

## 🚀 Inicio Rápido (5 minutos)

### 1. Verificar Sistema ROS

```bash
# Verificar que ROS está corriendo
rosnode list

# Debe mostrar al menos:
# /rosout
# /qnx2ros
# /ros2qnx
# /rosbridge_websocket
```

### 2. Verificar Tópicos

```bash
cd /home/ysc/jueying-teleop/multiplexor
chmod +x scripts/verify-ros-topics.sh
./scripts/verify-ros-topics.sh
```

**Salida esperada:**
```
✓ /cmd_vel - Control de velocidades
✓ /simple_cmd - Comandos discretos
✓ /complex_cmd - Comandos con parámetros
✓ /leg_odom - Odometría de patas
✓ /imu/data - Datos de IMU
✓ /joint_states - Estados de articulaciones
✓ /handle_state - Estado del handle
```

### 3. Iniciar Multiplexor

```bash
cd /home/ysc/jueying-teleop/multiplexor
npm start
```

**Salida esperada:**
```
[INFO]: Starting Jueying Multiplexor Server...
[INFO]: Connected to ROS at ws://localhost:9090
[INFO]: ✅ Multiplexor server running on port 8080
[INFO]: ✅ System ready for connections
```

### 4. Probar Conexión desde Frontend

Abrir navegador en: `http://192.168.1.103:3000`

**Credenciales de prueba:**
- Usuario: `operator1`
- Contraseña: `password123`

### 5. Enviar Primer Comando

Desde la interfaz web:
1. Click en botón "Conectar"
2. Ingresar credenciales
3. Click en "Stand" (el robot debería levantarse)

---

## 🔧 Solución Rápida de Problemas

### Problema: "Cannot connect to multiplexor"

```bash
# Verificar que está corriendo
ps aux | grep "node.*server.js"

# Verificar puerto
netstat -tlnp | grep 8080

# Reiniciar
pkill -f "node.*server.js"
npm start
```

### Problema: "Cannot infer topic type"

**CAUSA:** Usando tópicos incorrectos (versión anterior del multiplexor)

**SOLUCIÓN:** Asegurarse de usar esta versión actualizada con tópicos correctos:
- ✅ `/cmd_vel` (no `/pose_cmd`)
- ✅ `/simple_cmd` (no `/cmd`)
- ✅ `/complex_cmd` (no `/pose_cmd`)

### Problema: "Not authenticated"

```javascript
// Verificar que el frontend envía autenticación primero:
ws.send(JSON.stringify({
  type: 'auth',
  username: 'operator1',
  password: 'password123'
}));
```

### Problema: Robot no responde a comandos

```bash
# Verificar que ros2qnx está corriendo
rosnode list | grep ros2qnx

# Verificar conectividad al Motion Host
ping 192.168.1.120

# Ver si ros2qnx está recibiendo comandos
rostopic echo /cmd_vel
# Debe mostrar mensajes cuando envías comandos desde interfaz
```

### Problema: No recibo telemetría

```bash
# Verificar que qnx2ros está publicando
rostopic hz /leg_odom
rostopic hz /imu/data

# Si no hay publicaciones, reiniciar qnx2ros
rosnode kill /qnx2ros
# Luego reiniciar el nodo según tu sistema
```

---

## 📝 Comandos Útiles

### Estado del Sistema
```bash
# Ver todos los procesos relacionados
ps aux | grep -E "(roscore|rosbridge|multiplexor)"

# Ver conexiones de red
netstat -tlnp | grep -E "(9090|8080|3000)"

# Ver logs del multiplexor
tail -f /tmp/multiplexor.log

# Ver logs de rosbridge
tail -f /tmp/rosbridge.log
```

### Control del Multiplexor
```bash
# Iniciar
npm start

# Iniciar con logs detallados
LOG_LEVEL=debug npm start

# Iniciar en background
npm start > /tmp/multiplexor.log 2>&1 &

# Detener
pkill -f "node.*server.js"

# Reiniciar
pkill -f "node.*server.js" && npm start
```

### Testing
```bash
# Test de integración completo
node test/integration-test.js

# Test con usuario/servidor personalizado
TEST_USER=admin TEST_PASS=admin123 MULTIPLEXOR_URL=ws://192.168.1.103:8080 node test/integration-test.js
```

---

## 📊 Comandos del Robot

### Comandos Básicos
```javascript
// Levantarse
{ type: 'command', command: 'stand' }

// Sentarse
{ type: 'command', command: 'sit' }

// Modo amortiguación
{ type: 'command', command: 'damping' }
```

### Control de Movimiento
```javascript
// Avanzar a 0.5 m/s
{
  type: 'command',
  command: 'move',
  data: {
    velocity: {
      linear: { x: 0.5, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 }
    }
  }
}

// Girar en su lugar
{
  type: 'command',
  command: 'move',
  data: {
    velocity: {
      linear: { x: 0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0.5 }
    }
  }
}

// Detener
{ type: 'command', command: 'stop' }
```

### Ajustes de Postura
```javascript
// Cambiar altura del cuerpo a 0.3m
{
  type: 'command',
  command: 'body_height',
  data: { value: 0.3 }
}

// Inclinación adelante/atrás (pitch)
{
  type: 'command',
  command: 'pitch',
  data: { value: 0.2 }
}

// Inclinación lateral (roll)
{
  type: 'command',
  command: 'roll',
  data: { value: 0.1 }
}
```

---

## 🎯 Checklist de Despliegue

Antes de desplegar en producción:

- [ ] Cambiar `JWT_SECRET` en `.env`
- [ ] Cambiar contraseñas en `config/users.js`
- [ ] Ajustar `LOG_LEVEL` a `info` o `warn`
- [ ] Verificar permisos de archivos
- [ ] Configurar systemd service (opcional)
- [ ] Habilitar firewall para puerto 8080
- [ ] Documentar IPs y puertos en red
- [ ] Crear backup del sistema anterior
- [ ] Probar todos los comandos críticos
- [ ] Verificar telemetría en tiempo real

---

## 📞 Soporte

**Logs importantes:**
- Multiplexor: `/tmp/multiplexor.log`
- Rosbridge: `/tmp/rosbridge.log`
- ROS: `~/.ros/log/`

**Archivos de configuración:**
- Multiplexor: `/home/ysc/jueying-teleop/multiplexor/.env`
- Usuarios: `/home/ysc/jueying-teleop/multiplexor/config/users.js`
- Tópicos: `/home/ysc/jueying-teleop/multiplexor/config/ros-topics.js`

**Para reportar problemas:**
1. Capturar logs completos
2. Capturar salida de `rostopic list`
3. Capturar salida de `rosnode list`
4. Describir el comportamiento esperado vs. actual

---

**Versión:** 2.0.0  
**Última actualización:** Octubre 2025
