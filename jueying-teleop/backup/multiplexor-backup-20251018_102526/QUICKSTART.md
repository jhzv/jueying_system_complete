# 🚀 Quick Start - Multiplexor v2.1

Guía rápida para poner en marcha el multiplexor en menos de 5 minutos.

## ✅ Pre-requisitos

Verificar que estén activos:
```bash
# 1. ROS Master
roscore &

# 2. Rosbridge  
roslaunch rosbridge_server rosbridge_websocket.launch &

# 3. Verificar nodos del robot
rosnode list | grep -E "(qnx2ros|ros2qnx)"
```

## 📦 Instalación Express

```bash
# 1. Ir al directorio
cd /home/ysc/jueying-teleop/

# 2. Backup del anterior (si existe)
[ -d "multiplexor" ] && mv multiplexor multiplexor-old

# 3. Copiar proyecto
cp -r /path/to/multiplexor-v2.1-hybrid multiplexor
cd multiplexor

# 4. Instalar
npm install

# 5. Configurar
cp .env.example .env
nano .env  # Verificar IPs del robot

# 6. Crear logs
mkdir -p logs
```

## 🎮 Iniciar Multiplexor

```bash
cd /home/ysc/jueying-teleop/multiplexor
npm start
```

Deberías ver:
```
============================================================
Jueying Lite3 Multiplexor v2.1 - Hybrid ROS + UDP
============================================================
[UDP] Client ready. Target: 192.168.1.120:43893
[ROS] Connected to rosbridge
[SERVER] WebSocket listening on port 8080
[SERVER] Ready to accept connections
============================================================
```

## 🌐 Acceder desde Frontend

1. **Iniciar frontend (si no está activo):**
```bash
cd /home/ysc/jueying-teleop/frontend
python3 -m http.server 3000 &
```

2. **Abrir navegador:**
```
http://192.168.1.103:3000/jueying_control_complete.html
```

3. **Login:**
- Usuario: `operator1`
- Password: `password123`

## ⚡ Comandos Útiles

```bash
# Ver logs en tiempo real
tail -f logs/multiplexor-$(date +%Y-%m-%d).log

# Detener multiplexor
pkill -f "node.*server.js"

# Reiniciar
npm start

# Ver estado del sistema
~/status-jueying-teleop.sh

# Probar UDP
node tests/test_udp.js
```

## 🐛 Troubleshooting Rápido

### No conecta a rosbridge
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

### Puerto ocupado
```bash
pkill -f "node.*server.js"
npm start
```

### Robot no responde
1. Verificar IP en `.env`: `ROBOT_MOTION_HOST=192.168.1.120`
2. Ping al robot: `ping 192.168.1.120`
3. Ver logs: `tail -f logs/multiplexor-*.log`

## 📚 Más Info

- **Instalación completa:** Ver `INSTALL.md`
- **Documentación:** Ver `README.md`
- **Soporte:** jhosep.zapata@skaler.co

---

**¡Listo para controlar el robot! 🤖**
