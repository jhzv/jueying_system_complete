#!/bin/bash

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SERVICES=(
    "jueying-message-transformer"
    "jueying-rosbridge"
    "jueying-mediamtx"
    "jueying-multiplexor"
    "jueying-frontend"
)

case "$1" in
    start)
        echo -e "${BLUE}🚀 Iniciando servicios Jueying...${NC}"
        for service in "${SERVICES[@]}"; do
            echo -e "   Iniciando ${service}..."
            sudo systemctl start ${service}.service
            sleep 2
        done
        echo -e "${GREEN}✅ Servicios iniciados${NC}"
        ;;
        
    stop)
        echo -e "${YELLOW}🛑 Deteniendo servicios Jueying...${NC}"
        for service in "${SERVICES[@]}"; do
            sudo systemctl stop ${service}.service
        done
        echo -e "${GREEN}✅ Servicios detenidos${NC}"
        ;;
        
    restart)
        echo -e "${YELLOW}🔄 Reiniciando servicios Jueying...${NC}"
        $0 stop
        sleep 3
        $0 start
        ;;
        
    status)
        echo -e "${BLUE}📊 Estado de servicios Jueying:${NC}"
        echo ""
        systemctl status jueying-*.service --no-pager | grep -E "(●|Active:|Main PID:)" | head -30
        ;;
        
    logs)
        service=${2:-jueying-multiplexor}
        echo -e "${BLUE}📝 Logs de ${service}:${NC}"
        sudo journalctl -u ${service}.service -f
        ;;
        
    enable)
        echo -e "${BLUE}✅ Habilitando auto-inicio...${NC}"
        for service in "${SERVICES[@]}"; do
            sudo systemctl enable ${service}.service
        done
        echo -e "${GREEN}✅ Auto-inicio habilitado${NC}"
        ;;
        
    disable)
        echo -e "${YELLOW}❌ Deshabilitando auto-inicio...${NC}"
        for service in "${SERVICES[@]}"; do
            sudo systemctl disable ${service}.service
        done
        echo -e "${GREEN}✅ Auto-inicio deshabilitado${NC}"
        ;;
        
    *)
        echo "Uso: $0 {start|stop|restart|status|logs [servicio]|enable|disable}"
        echo ""
        echo "Servicios disponibles:"
        for service in "${SERVICES[@]}"; do
            echo "  - $service"
        done
        exit 1
        ;;
esac
