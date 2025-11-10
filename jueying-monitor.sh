#!/bin/bash

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

while true; do
    clear
    echo "======================================================"
    echo "  🤖 MONITOR JUEYING LITE3 - $(date '+%H:%M:%S')"
    echo "======================================================"
    echo ""
    
    # Servicios Systemd
    echo "📊 Servicios Systemd:"
    systemctl is-active jueying-message-transformer >/dev/null 2>&1 && echo -e "  ${GREEN}✓${NC} Message Transformer" || echo -e "  ${RED}✗${NC} Message Transformer"
    systemctl is-active jueying-rosbridge >/dev/null 2>&1 && echo -e "  ${GREEN}✓${NC} ROS Bridge" || echo -e "  ${RED}✗${NC} ROS Bridge"
    systemctl is-active jueying-mediamtx >/dev/null 2>&1 && echo -e "  ${GREEN}✓${NC} MediaMTX (Video)" || echo -e "  ${RED}✗${NC} MediaMTX (Video)"
    systemctl is-active jueying-multiplexor >/dev/null 2>&1 && echo -e "  ${GREEN}✓${NC} Multiplexor" || echo -e "  ${RED}✗${NC} Multiplexor"
    systemctl is-active jueying-frontend >/dev/null 2>&1 && echo -e "  ${GREEN}✓${NC} Frontend" || echo -e "  ${RED}✗${NC} Frontend"
    
    echo ""
    echo "🎥 Video:"
    if pgrep -f "ffmpeg.*video2" > /dev/null; then
        echo -e "  ${GREEN}✓${NC} FFmpeg capturando cámara"
    else
        echo -e "  ${YELLOW}⚠${NC} FFmpeg no está capturando"
    fi
    
    if [ -e /dev/video2 ]; then
        echo -e "  ${GREEN}✓${NC} Cámara disponible (/dev/video2)"
    else
        echo -e "  ${RED}✗${NC} Cámara NO disponible"
    fi
    
    echo ""
    echo "🌐 Red:"
    echo "  LAN: 192.168.1.103"
    echo "  Tailscale: $(tailscale ip -4 2>/dev/null || echo 'No disponible')"
    
    echo ""
    echo "📊 Recursos:"
    echo "  CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' 2>/dev/null || echo 'N/A')"
    echo "  RAM: $(free -h | awk '/^Mem:/ {print $3 "/" $2}')"
    
    echo ""
    echo "======================================================"
    echo "Presiona Ctrl+C para salir | Actualiza cada 3s"
    
    sleep 3
done
