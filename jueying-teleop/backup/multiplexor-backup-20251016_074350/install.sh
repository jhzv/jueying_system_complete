#!/bin/bash
# install.sh - Script de instalación del multiplexor actualizado

set -e

echo "======================================"
echo "Jueying Multiplexor - Instalación"
echo "======================================"
echo ""

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Verificar Node.js
echo -n "Verificando Node.js..."
if ! command -v node &> /dev/null; then
    echo -e "${RED}✗${NC}"
    echo "Node.js no está instalado. Por favor instalar Node.js 18+ primero."
    exit 1
fi

NODE_VERSION=$(node -v | cut -d'v' -f2 | cut -d'.' -f1)
if [ "$NODE_VERSION" -lt 18 ]; then
    echo -e "${RED}✗${NC}"
    echo "Node.js version $NODE_VERSION detectada. Se requiere versión 18 o superior."
    exit 1
fi
echo -e "${GREEN}✓${NC} $(node -v)"

# Verificar npm
echo -n "Verificando npm..."
if ! command -v npm &> /dev/null; then
    echo -e "${RED}✗${NC}"
    echo "npm no está instalado."
    exit 1
fi
echo -e "${GREEN}✓${NC} $(npm -v)"

# Directorio de instalación
INSTALL_DIR="/home/ysc/jueying-teleop/multiplexor"
echo ""
echo "Directorio de instalación: $INSTALL_DIR"

# Backup si existe instalación anterior
if [ -d "$INSTALL_DIR" ]; then
    BACKUP_DIR="${INSTALL_DIR}-backup-$(date +%Y%m%d-%H%M%S)"
    echo -n "Haciendo backup de instalación anterior..."
    mv "$INSTALL_DIR" "$BACKUP_DIR"
    echo -e "${GREEN}✓${NC}"
    echo "Backup guardado en: $BACKUP_DIR"
fi

# Copiar archivos
echo -n "Copiando archivos..."
mkdir -p "$(dirname $INSTALL_DIR)"
cp -r "$(dirname $0)" "$INSTALL_DIR"
echo -e "${GREEN}✓${NC}"

# Instalar dependencias
cd "$INSTALL_DIR"
echo "Instalando dependencias de Node.js..."
npm install --production

if [ $? -ne 0 ]; then
    echo -e "${RED}✗${NC} Error al instalar dependencias"
    exit 1
fi
echo -e "${GREEN}✓${NC} Dependencias instaladas"

# Configurar .env
if [ ! -f "$INSTALL_DIR/.env" ]; then
    echo -n "Creando archivo de configuración..."
    cp "$INSTALL_DIR/.env.example" "$INSTALL_DIR/.env"
    
    # Solicitar JWT Secret
    echo ""
    read -p "Ingrese un JWT Secret (o Enter para usar uno aleatorio): " JWT_SECRET
    if [ -z "$JWT_SECRET" ]; then
        JWT_SECRET=$(openssl rand -base64 32 | tr -d "=+/" | cut -c1-32)
        echo "JWT Secret generado automáticamente"
    fi
    sed -i "s/your-super-secret-jwt-key-change-this-in-production/$JWT_SECRET/" "$INSTALL_DIR/.env"
    
    echo -e "${GREEN}✓${NC} Archivo .env creado"
    echo ""
    echo -e "${YELLOW}IMPORTANTE:${NC} Revisar y ajustar configuración en $INSTALL_DIR/.env"
fi

# Crear logs directory
echo -n "Creando directorio de logs..."
mkdir -p /tmp/jueying-logs
chmod 755 /tmp/jueying-logs
echo -e "${GREEN}✓${NC}"

# Verificar conectividad ROS
echo ""
echo "Verificando servicios ROS..."

# Verificar roscore
if ! pgrep -x "roscore" > /dev/null; then
    echo -e "${YELLOW}⚠${NC} roscore no está corriendo"
    echo "  Por favor iniciar roscore antes de usar el multiplexor"
else
    echo -e "${GREEN}✓${NC} roscore está corriendo"
fi

# Verificar rosbridge
if ! netstat -tlnp 2>/dev/null | grep -q ":9090 "; then
    echo -e "${YELLOW}⚠${NC} rosbridge no está escuchando en puerto 9090"
    echo "  Por favor iniciar rosbridge_server antes de usar el multiplexor"
else
    echo -e "${GREEN}✓${NC} rosbridge está corriendo en puerto 9090"
fi

# Test de conexión
echo ""
echo "======================================"
echo "Instalación completada"
echo "======================================"
echo ""
echo "Siguiente pasos:"
echo "1. Revisar configuración: nano $INSTALL_DIR/.env"
echo "2. Verificar usuarios: nano $INSTALL_DIR/config/users.js"
echo "3. Iniciar el multiplexor:"
echo "   cd $INSTALL_DIR"
echo "   npm start"
echo ""
echo "O usar el script de inicio del sistema:"
echo "   ~/start-jueying-native.sh"
echo ""
echo -e "${GREEN}¡Listo para usar!${NC}"
echo ""
