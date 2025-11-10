#!/bin/bash

echo "📊 Estado del Sistema de Teleoperación"
echo "========================================"

echo ""
echo "🤖 ROS:"
pgrep roscore >/dev/null && echo "✅ ROS Master (PID: $(pgrep roscore))" || echo "❌ ROS Master detenido"
pgrep -f rosbridge >/dev/null && echo "✅ Rosbridge (PID: $(pgrep -f rosbridge))" || echo "❌ Rosbridge detenido"

echo ""
echo "🐳 Contenedores Docker:"
sudo docker ps --filter name=jueying --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"

echo ""
echo "🌐 Frontend:"
pgrep -f "http.server 3000" >/dev/null && echo "✅ Frontend (PID: $(pgrep -f 'http.server 3000'))" || echo "❌ Frontend detenido"

echo ""
echo "🔌 Conectividad de Puertos:"
for port in 9090 8080 3000; do
    if timeout 2 bash -c "</dev/tcp/localhost/$port" 2>/dev/null; then
        echo "✅ Puerto $port abierto"
    else
        echo "❌ Puerto $port cerrado"
    fi
done

echo ""
echo "💾 Recursos del Sistema:"
echo "CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)% usado"
echo "RAM: $(free -h | grep Mem | awk '{print $3"/"$2}')"
echo "Disk: $(df -h /home | tail -1 | awk '{print $3"/"$2" ("$5" usado)"}')"
