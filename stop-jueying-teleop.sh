#!/bin/bash

echo "🛑 Deteniendo Sistema de Teleoperación Jueying Lite3"

# Detener frontend
echo "Deteniendo frontend..."
pkill -f "http.server 3000"

# Detener multiplexor
echo "Deteniendo multiplexor..."
sudo docker stop jueying-multiplexor 2>/dev/null || true

# Detener rosbridge
echo "Deteniendo rosbridge..."
pkill -f "rosbridge"

# Detener ROS (opcional - comentado por seguridad)
# echo "Deteniendo ROS Master..."
# pkill roscore

echo "✅ Sistema detenido"

