#!/bin/bash

echo "🔍 Test Simple de Rosbridge"
echo "============================"

# Probar que el puerto esté abierto
echo ""
echo "1️⃣ Verificando puerto 9090..."
if timeout 2 bash -c "</dev/tcp/localhost/9090" 2>/dev/null; then
    echo "✅ Puerto 9090 abierto"
else
    echo "❌ Puerto 9090 cerrado"
    echo "   Inicia rosbridge: roslaunch rosbridge_server rosbridge_websocket.launch"
    exit 1
fi

# Probar HTTP
echo ""
echo "2️⃣ Probando HTTP de rosbridge..."
response=$(curl -s http://localhost:9090 | head -1)
if [[ $response == *"AutobahnPython"* ]]; then
    echo "✅ Rosbridge responde correctamente"
else
    echo "❌ Rosbridge no responde correctamente"
fi

# Listar tópicos ROS
echo ""
echo "3️⃣ Tópicos ROS disponibles:"
rostopic list 2>/dev/null || echo "❌ No se puede listar tópicos"

# Verificar tópicos específicos para Jueying
echo ""
echo "4️⃣ Tópicos importantes:"
for topic in /cmd_vel /pose_cmd /rosout /robot_status; do
    if rostopic list 2>/dev/null | grep -q "^${topic}$"; then
        echo "✅ $topic"
    else
        echo "○ $topic (no disponible aún)"
    fi
done

echo ""
echo "✅ Test completado"

