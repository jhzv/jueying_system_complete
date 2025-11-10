# Guía de Instalación - Multiplexor v2.1 Híbrido

Guía detallada para instalar y configurar el multiplexor en el Jetson Xavier NX.

## 📋 Requisitos Previos

### Sistema
- ✅ Ubuntu 20.04 LTS (Jetson Xavier NX)
- ✅ ROS Melodic instalado
- ✅ Node.js 18+ (instalar vía NVM)

### Servicios Activos
```bash
# Verificar ROS
roscore &
rosnode list

# Verificar rosbridge
roslaunch rosbridge_server rosbridge_websocket.launch &
netstat -tlnp | grep 9090

# Verificar nodos del robot
rosnode list | grep -E "(qnx2ros|ros2qnx|nx2app)"
```

## 🚀 Instalación Paso a Paso

### 1. Instalar Node.js (si no está instalado)

```bash
# Instalar NVM
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash

# Recargar terminal
source ~/.bashrc

# Instalar Node.js 18
nvm install 18
nvm use 18
nvm alias default 18

# Verificar instalación
node --version  # Debe mostrar v18.x.x
npm --version   # Debe mostrar 9.x.x o superior
```

### 2. Preparar Directorio

```bash
# Ir al directorio de proyectos
cd /home/ysc/jueying-teleop/

# Backup del multiplexor anterior (si existe)
if [ -d "multiplexor" ]; then
  mv multiplexor multiplexor-backup-$(date +%Y%m%d_%H%M%S)
  echo "✅ Backup creado"
fi

# Crear nuevo directorio
mkdir -p multiplexor
cd multiplexor
```

### 3. Copiar Archivos del Proyecto

**Opción A: Si tienes el tarball**
```bash
cd /home/ysc/
tar -xzf multiplexor-v2.1-hybrid.tar.gz
mv multiplexor-v2.1-hybrid /home/ysc/jueying-teleop/multiplexor
cd /home/ysc/jueying-teleop/multiplexor
```

**Opción B: Clonar desde repositorio**
```bash
cd /home/ysc/jueying-teleop/
git clone <URL_DEL_REPO> multiplexor
cd multiplexor
```

**Opción C: Copiar desde USB/red**
```bash
# Montar USB si es necesario
sudo mount /dev/sda1 /mnt/usb

# Copiar archivos
cp -r /mnt/usb/multiplexor-v2.1-hybrid/* /home/ysc/jueying-teleop/multiplexor/

cd /home/ysc/jueying-teleop/multiplexor
```

### 4. Instalar Dependencias

```bash
# Asegurar que estamos en el directorio correcto
cd /home/ysc/jueying-teleop/multiplexor

# Instalar dependencias npm
npm install

# Esto instalará:
# - ws (WebSocket)
# - roslibjs (ROS client)
# - bcrypt (passwords)
# - jsonwebtoken (JWT)
# - winston (logging)
# - dotenv (configuración)
```

### 5. Configurar Variables de Entorno

```bash
# Copiar ejemplo
cp .env.example .env

# Editar configuración
nano .env
```

**Configurar estos valores críticos:**
```bash
# ROBOT - Verificar IPs correctas
ROBOT_MOTION_HOST=192.168.1.120    # Motion Host (RK3588)
ROBOT_PERCEPTION_HOST=192.168.1.103  # Este Jetson

# SEGURIDAD - CAMBIAR JWT_SECRET
JWT_SECRET=TuSecretoSuperSeguroAquí2024

# LOGS
LOG_LEVEL=info  # Usar "debug" para más detalle
```

Guardar con `Ctrl+O`, `Enter`, `Ctrl+X`

### 6. Crear Directorio de Logs

```bash
mkdir -p logs
chmod 755 logs
```

### 7. Verificar Conectividad al Robot

```bash
# Ping al Motion Host
ping -c 3 192.168.1.120

# Si no responde, verificar red
ifconfig
route -n

# Verificar tópicos ROS
rostopic list
rostopic hz /leg_odom
```

## ✅ Primera Ejecución (Prueba)

### 1. Iniciar Servicios ROS (si no están activos)

```bash
# Terminal 1: ROS Master
roscore

# Terminal 2: Rosbridge
roslaunch rosbridge_server rosbridge_websocket.launch

# Terminal 3: Nodos del robot (si no están en autostart)
# (Normalmente ya están corriendo)
```

### 2. Iniciar Multiplexor en Modo Debug

```bash
cd /home/ysc/jueying-teleop/multiplexor

# Primera ejecución en foreground para ver errores
npm start
```

**Deberías ver:**
```
============================================================
Jueying Lite3 Multiplexor v2.1 - Hybrid ROS + UDP
============================================================
12:34:56 info: [UDP] Initializing client...
12:34:56 info: [UDP] Client ready. Target: 192.168.1.120:43893
12:34:56 info: [UDP] Heartbeat started (250ms)
12:34:56 info: [UDP] Mode changed to: pose
12:34:56 info: [ROS] Connecting to rosbridge: ws://localhost:9090
12:34:57 info: [ROS] Connected to rosbridge
12:34:57 info: [ROS] Subscribed to telemetry topics
12:34:57 info: [SERVER] WebSocket listening on port 8080
12:34:57 info: [SERVER] Ready to accept connections
============================================================
```

### 3. Probar Conexión desde Frontend

**En otra terminal del Jetson:**
```bash
cd /home/ysc/jueying-teleop/frontend
python3 -m http.server 3000 &

# Abrir navegador en:
# http://192.168.1.103:3000/jueying_control_complete.html
```

**O desde tu computadora:**
```
http://192.168.1.103:3000/jueying_control_complete.html
```

**Credenciales de prueba:**
- Usuario: `operator1`
- Password: `password123`

### 4. Verificar Logs

```bash
# Ver logs en tiempo real
tail -f logs/multiplexor-$(date +%Y-%m-%d).log

# Ver solo errores
tail -f logs/error-$(date +%Y-%m-%d).log

# Buscar conexiones
grep "Connected" logs/multiplexor-*.log
```

## 🔧 Solución de Problemas

### Error: "Cannot connect to rosbridge"

```bash
# Verificar que rosbridge está corriendo
rosnode list | grep rosbridge

# Si no está, iniciar:
roslaunch rosbridge_server rosbridge_websocket.launch

# Verificar puerto
netstat -tlnp | grep 9090
```

### Error: "UDP send error"

```bash
# Verificar conectividad al Motion Host
ping 192.168.1.120

# Verificar que no hay firewall
sudo iptables -L

# Verificar puerto UDP
nc -u -v 192.168.1.120 43893
```

### Error: "EADDRINUSE" (puerto en uso)

```bash
# Ver qué está usando el puerto 8080
sudo netstat -tlnp | grep 8080

# Matar proceso
sudo kill -9 <PID>

# O cambiar puerto en .env
nano .env
# PORT=8081
```

### El robot no responde a comandos

1. Verificar modo actual:
```bash
# Ver logs
tail -f logs/multiplexor-*.log | grep "Mode changed"
```

2. Cambiar a modo correcto desde frontend:
```javascript
// En consola del navegador
ws.send(JSON.stringify({
  type: 'command',
  command: 'set_mode',
  data: { mode: 'move' }
}));
```

3. Verificar heartbeat:
```bash
# Monitorear tráfico UDP
sudo tcpdump -i any -n udp port 43893 -c 10
```

## 🚀 Configurar Inicio Automático (Opcional)

### Opción A: Script de Inicio Manual

Crear `/home/ysc/start-jueying-multiplexor.sh`:
```bash
#!/bin/bash
cd /home/ysc/jueying-teleop/multiplexor
npm start > /tmp/multiplexor-startup.log 2>&1 &
echo "Multiplexor iniciado (PID: $!)"
```

Hacer ejecutable:
```bash
chmod +x ~/start-jueying-multiplexor.sh
```

### Opción B: Servicio Systemd

```bash
# Copiar servicio
sudo cp systemd/jueying-multiplexor.service /etc/systemd/system/

# Editar rutas si es necesario
sudo nano /etc/systemd/system/jueying-multiplexor.service

# Recargar systemd
sudo systemctl daemon-reload

# Habilitar inicio automático
sudo systemctl enable jueying-multiplexor

# Iniciar ahora
sudo systemctl start jueying-multiplexor

# Ver estado
sudo systemctl status jueying-multiplexor

# Ver logs
sudo journalctl -u jueying-multiplexor -f
```

## 📊 Verificar Instalación Completa

```bash
# Ejecutar script de verificación
./scripts/verify_installation.sh
```

O manualmente:
```bash
# 1. Verificar Node.js
node --version

# 2. Verificar dependencias
npm list --depth=0

# 3. Verificar ROS
rosnode list

# 4. Verificar rosbridge
rostopic list

# 5. Verificar multiplexor
ps aux | grep "node.*server.js"

# 6. Verificar puerto
netstat -tlnp | grep 8080

# 7. Verificar logs
ls -lh logs/

# 8. Probar autenticación
node -e "require('./config/users').listUsers()"
```

## 🎯 Siguiente Paso

Una vez instalado y verificado:
1. ✅ Abrir frontend en navegador
2. ✅ Autenticarse con operator1/password123
3. ✅ Solicitar control
4. ✅ Cambiar a modo MOVE
5. ✅ Probar movimiento con joystick
6. ✅ Cambiar a modo POSE
7. ✅ Probar acción sit_stand

## 📞 Soporte

Si encuentras problemas:
1. Revisar logs: `tail -f logs/multiplexor-*.log`
2. Verificar estado: `systemctl status jueying-multiplexor`
3. Contactar: jhosep.zapata@skaler.co

---

**Instalación completada** ✅
