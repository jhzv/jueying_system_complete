#!/bin/bash

echo "=========================================="
echo "🤖 Sistema de Teleoperación Jueying Lite3"
echo "=========================================="

# 1. Iniciar ROS Master
echo ""
echo "1️⃣ Verificando ROS Master..."
if ! pgrep -x "roscore" > /dev/null; then
    echo "   Iniciando ROS Master..."
    source /opt/ros/noetic/setup.bash
    roscore > /tmp/roscore.log 2>&1 &
    sleep 3
    echo "   ✅ ROS Master iniciado"
else
    echo "   ✅ ROS Master ya corriendo (PID: $(pgrep roscore))"
fi

# 2. Iniciar rosbridge
echo ""
echo "2️⃣ Verificando rosbridge..."
if ! pgrep -f "rosbridge" > /dev/null; then
    echo "   Iniciando rosbridge..."
    source /opt/ros/noetic/setup.bash
    roslaunch rosbridge_server rosbridge_websocket.launch > /tmp/rosbridge.log 2>&1 &
    sleep 5
    echo "   ✅ Rosbridge iniciado"
else
    echo "   ✅ Rosbridge ya corriendo (PID: $(pgrep -f rosbridge))"
fi

# Verificar que rosbridge responde
echo "   🔍 Verificando conectividad rosbridge..."
timeout 10 bash -c 'until curl -f http://localhost:9090 2>/dev/null; do sleep 1; done' && \
    echo "   ✅ Rosbridge respondiendo en puerto 9090" || \
    echo "   ⚠️  Rosbridge no responde (revisar logs: tail -f /tmp/rosbridge.log)"

# 3. Iniciar Multiplexor (contenedor)
echo ""
echo "3️⃣ Verificando Multiplexor..."
if sudo docker ps | grep -q jueying-teleop-containers-multiplexor; then
    echo "   ✅ Multiplexor ya corriendo"
else
    echo "   Deteniendo contenedor anterior si existe..."
    sudo docker stop jueying-multiplexor 2>/dev/null || true
    sudo docker rm jueying-multiplexor 2>/dev/null || true
    
    echo "   Iniciando contenedor multiplexor..."
    sudo docker run -d \
        --name jueying-multiplexor \
        --network host \
        -e NODE_ENV=production \
        -e ROSBRIDGE_URL=ws://localhost:9090 \
        -e ROBOT_MOTION_HOST=192.168.1.120 \
        -e ROBOT_PERCEPTION_HOST=192.168.1.103 \
        --restart unless-stopped \
        jueying-teleop-containers-multiplexor:latest
    
    sleep 3
    echo "   ✅ Multiplexor iniciado"
fi

# 4. Iniciar Frontend (sin contenedor)
echo ""
echo "4️⃣ Verificando Frontend..."
if pgrep -f "http.server 3000" > /dev/null; then
    echo "   ✅ Frontend ya corriendo (PID: $(pgrep -f 'http.server 3000'))"
else
    echo "   Iniciando servidor web frontend..."
    cd /home/ysc/jueying-teleop/frontend
    python3 -m http.server 3000 > /tmp/frontend.log 2>&1 &
    sleep 2
    echo "   ✅ Frontend iniciado"
fi

# 5. Verificación final
echo ""
echo "=========================================="
echo "📊 VERIFICACIÓN DEL SISTEMA"
echo "=========================================="

echo ""
echo "🤖 ROS Services:"
pgrep roscore >/dev/null && echo "   ✅ ROS Master (PID: $(pgrep roscore))" || echo "   ❌ ROS Master"
pgrep -f rosbridge >/dev/null && echo "   ✅ Rosbridge (PID: $(pgrep -f rosbridge))" || echo "   ❌ Rosbridge"

echo ""
echo "🐳 Docker Containers:"
sudo docker ps --filter name=jueying-multiplexor --format "   ✅ {{.Names}} - {{.Status}}"

echo ""
echo "🌐 Frontend:"
pgrep -f "http.server 3000" >/dev/null && echo "   ✅ Frontend (PID: $(pgrep -f 'http.server 3000'))" || echo "   ❌ Frontend"

echo ""
echo "🔌 Puertos:"
for port in 9090 8080 3000; do
    if timeout 2 bash -c "</dev/tcp/localhost/$port" 2>/dev/null; then
        echo "   ✅ Puerto $port abierto"
    else
        echo "   ❌ Puerto $port cerrado"
    fi
done

echo ""
echo "=========================================="
echo "🎯 ACCESO AL SISTEMA"
echo "=========================================="
echo ""
echo "   🌐 Interfaz Web:"
echo "      http://192.168.1.103:3000/jueying_control_complete.html"
echo ""
echo "   🔧 API Multiplexor:"
echo "      http://192.168.1.103:8080"
echo ""
echo "   📡 Rosbridge WebSocket:"
echo "      ws://192.168.1.103:9090"
echo ""
echo "=========================================="
echo ""
echo "📝 Ver logs:"
echo "   ROS Master:   tail -f /tmp/roscore.log"
echo "   Rosbridge:    tail -f /tmp/rosbridge.log"
echo "   Multiplexor:  sudo docker logs -f jueying-multiplexor"
echo "   Frontend:     tail -f /tmp/frontend.log"
echo ""
echo "🛑 Detener sistema: ./stop-jueying-teleop.sh"
echo ""

