#!/bin/bash
#
# Script Maestro de Corrección - Multiplexor v2.1
# Aplica automáticamente las 3 correcciones necesarias
#
# Uso: bash fix_all.sh
#

set -e  # Salir si hay error

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}============================================================${NC}"
echo -e "${BLUE}  Multiplexor v2.1 - Script de Corrección Automática${NC}"
echo -e "${BLUE}============================================================${NC}"
echo ""

# Variables
BASE_DIR="/home/ysc/jueying-teleop"
MULTIPLEXOR_DIR="$BASE_DIR/multiplexor"
FRONTEND_DIR="$BASE_DIR/frontend"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Verificar que estamos en el lugar correcto
if [ ! -d "$BASE_DIR" ]; then
    echo -e "${RED}❌ Error: Directorio $BASE_DIR no existe${NC}"
    exit 1
fi

cd "$BASE_DIR"
echo -e "${GREEN}✅ Directorio base: $BASE_DIR${NC}"
echo ""

# ============================================================
# CORRECCIÓN 1: REORGANIZAR DIRECTORIOS
# ============================================================
echo -e "${YELLOW}[1/3] Reorganizando directorios...${NC}"

if [ -d "$MULTIPLEXOR_DIR" ]; then
    # Hacer backup
    BACKUP_DIR="multiplexor-backup-$TIMESTAMP"
    echo "  📦 Creando backup: $BACKUP_DIR"
    mv "$MULTIPLEXOR_DIR" "$BACKUP_DIR"
    
    # Buscar el directorio correcto
    if [ -d "$BACKUP_DIR/multiplexor-v2.1-hybrid" ]; then
        echo "  📂 Moviendo multiplexor-v2.1-hybrid..."
        mv "$BACKUP_DIR/multiplexor-v2.1-hybrid" "$MULTIPLEXOR_DIR"
        echo -e "${GREEN}  ✅ Directorios reorganizados${NC}"
    else
        echo -e "${RED}  ❌ Error: No se encontró multiplexor-v2.1-hybrid${NC}"
        echo "  Por favor extraer manualmente el tar.gz"
        exit 1
    fi
else
    echo -e "${YELLOW}  ⚠️  No existe $MULTIPLEXOR_DIR${NC}"
    echo "  Buscando tar.gz..."
    
    if [ -f "$HOME/multiplexor-v2.1-hybrid.tar.gz" ]; then
        echo "  📦 Extrayendo tar.gz..."
        tar -xzf "$HOME/multiplexor-v2.1-hybrid.tar.gz"
        mv multiplexor-v2.1-hybrid "$MULTIPLEXOR_DIR"
        echo -e "${GREEN}  ✅ Multiplexor extraído${NC}"
    else
        echo -e "${RED}  ❌ Error: No se encontró multiplexor-v2.1-hybrid.tar.gz${NC}"
        exit 1
    fi
fi

# Verificar estructura
if [ ! -f "$MULTIPLEXOR_DIR/package.json" ]; then
    echo -e "${RED}❌ Error: package.json no encontrado${NC}"
    exit 1
fi

echo ""

# ============================================================
# CORRECCIÓN 2: ARREGLAR BUG UDP
# ============================================================
echo -e "${YELLOW}[2/3] Corrigiendo bug UDP...${NC}"

UDP_CLIENT_FILE="$MULTIPLEXOR_DIR/src/udp/client.js"

if [ ! -f "$UDP_CLIENT_FILE" ]; then
    echo -e "${RED}  ❌ Error: $UDP_CLIENT_FILE no existe${NC}"
    exit 1
fi

# Hacer backup del archivo original
cp "$UDP_CLIENT_FILE" "$UDP_CLIENT_FILE.backup"

# Aplicar corrección (agregar llaves en línea 7)
sed -i '7s/const UDP_COMMANDS = require/const { UDP_COMMANDS } = require/' "$UDP_CLIENT_FILE"

# Verificar corrección
if grep -q "const { UDP_COMMANDS }" "$UDP_CLIENT_FILE"; then
    echo -e "${GREEN}  ✅ Bug UDP corregido${NC}"
    echo "  📝 Backup guardado: $UDP_CLIENT_FILE.backup"
else
    echo -e "${RED}  ❌ Error: No se pudo aplicar corrección${NC}"
    echo "  Aplicar manualmente: const { UDP_COMMANDS } = require('./constants');"
    exit 1
fi

echo ""

# ============================================================
# CORRECCIÓN 3: INSTALAR FRONTEND COMPATIBLE
# ============================================================
echo -e "${YELLOW}[3/3] Instalando frontend compatible...${NC}"

# Buscar el frontend nuevo
FRONTEND_NEW="jueying_control_v2.1.html"

if [ -f "$HOME/$FRONTEND_NEW" ]; then
    FRONTEND_SOURCE="$HOME/$FRONTEND_NEW"
elif [ -f "./$FRONTEND_NEW" ]; then
    FRONTEND_SOURCE="./$FRONTEND_NEW"
elif [ -f "/tmp/$FRONTEND_NEW" ]; then
    FRONTEND_SOURCE="/tmp/$FRONTEND_NEW"
else
    echo -e "${YELLOW}  ⚠️  Frontend nuevo no encontrado${NC}"
    echo "  Ubicaciones buscadas:"
    echo "    - $HOME/$FRONTEND_NEW"
    echo "    - ./$FRONTEND_NEW"
    echo "    - /tmp/$FRONTEND_NEW"
    echo ""
    echo "  Continuando sin actualizar frontend..."
    FRONTEND_SOURCE=""
fi

if [ -n "$FRONTEND_SOURCE" ]; then
    # Crear directorio frontend si no existe
    mkdir -p "$FRONTEND_DIR"
    
    # Hacer backup del frontend antiguo si existe
    if [ -f "$FRONTEND_DIR/jueying_control_complete.html" ]; then
        echo "  📦 Backup del frontend antiguo..."
        mv "$FRONTEND_DIR/jueying_control_complete.html" \
           "$FRONTEND_DIR/jueying_control_complete.html.old"
    fi
    
    # Copiar frontend nuevo
    cp "$FRONTEND_SOURCE" "$FRONTEND_DIR/"
    echo -e "${GREEN}  ✅ Frontend v2.1 instalado${NC}"
    echo "  📍 Ubicación: $FRONTEND_DIR/$FRONTEND_NEW"
else
    echo -e "${YELLOW}  ⚠️  Skipping frontend installation${NC}"
fi

echo ""

# ============================================================
# VERIFICACIÓN E INSTALACIÓN DE DEPENDENCIAS
# ============================================================
echo -e "${YELLOW}[*] Instalando dependencias...${NC}"

cd "$MULTIPLEXOR_DIR"

if [ ! -d "node_modules" ]; then
    echo "  📦 Ejecutando npm install..."
    npm install
    echo -e "${GREEN}  ✅ Dependencias instaladas${NC}"
else
    echo "  ℹ️  node_modules ya existe (usando dependencias existentes)"
fi

# Crear directorio de logs si no existe
mkdir -p logs

echo ""

# ============================================================
# RESUMEN
# ============================================================
echo -e "${BLUE}============================================================${NC}"
echo -e "${BLUE}  RESUMEN DE CORRECCIONES${NC}"
echo -e "${BLUE}============================================================${NC}"
echo ""
echo -e "${GREEN}✅ [1/3] Directorios reorganizados${NC}"
echo "     📁 $MULTIPLEXOR_DIR"
echo ""
echo -e "${GREEN}✅ [2/3] Bug UDP corregido${NC}"
echo "     📝 $UDP_CLIENT_FILE"
echo "     🔧 Línea 7: const { UDP_COMMANDS } = require('./constants');"
echo ""

if [ -n "$FRONTEND_SOURCE" ]; then
    echo -e "${GREEN}✅ [3/3] Frontend v2.1 instalado${NC}"
    echo "     📍 $FRONTEND_DIR/$FRONTEND_NEW"
else
    echo -e "${YELLOW}⚠️  [3/3] Frontend v2.1 NO instalado${NC}"
    echo "     Copiar manualmente $FRONTEND_NEW a $FRONTEND_DIR/"
fi

echo ""
echo -e "${BLUE}============================================================${NC}"
echo -e "${BLUE}  SIGUIENTE PASO${NC}"
echo -e "${BLUE}============================================================${NC}"
echo ""
echo "1. Iniciar multiplexor:"
echo -e "   ${GREEN}cd $MULTIPLEXOR_DIR${NC}"
echo -e "   ${GREEN}npm start${NC}"
echo ""
echo "2. Verificar que arranca sin errores:"
echo "   ✅ [UDP] Client ready"
echo "   ✅ [UDP] Heartbeat started"
echo "   ✅ [UDP] Mode changed to: pose  ← SIN ERROR"
echo "   ✅ [ROS] Connected to rosbridge"
echo "   ✅ [SERVER] WebSocket listening on port 8080"
echo ""
echo "3. Iniciar frontend:"
echo -e "   ${GREEN}cd $FRONTEND_DIR${NC}"
echo -e "   ${GREEN}python3 -m http.server 3000${NC}"
echo ""
echo "4. Abrir navegador:"
if [ -n "$FRONTEND_SOURCE" ]; then
    echo "   http://192.168.1.103:3000/$FRONTEND_NEW"
else
    echo "   http://192.168.1.103:3000/jueying_control_complete.html"
fi
echo ""
echo "5. Login:"
echo "   Usuario: operator1"
echo "   Password: password123"
echo ""
echo -e "${BLUE}============================================================${NC}"
echo ""
echo -e "${GREEN}🎉 Correcciones aplicadas exitosamente!${NC}"
echo ""

# ============================================================
# PREGUNTAR SI QUIERE INICIAR AHORA
# ============================================================
read -p "¿Iniciar el multiplexor ahora? (s/n): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[SsYy]$ ]]; then
    echo ""
    echo -e "${BLUE}Iniciando multiplexor...${NC}"
    echo ""
    
    cd "$MULTIPLEXOR_DIR"
    npm start
else
    echo ""
    echo "Para iniciar manualmente:"
    echo -e "  ${GREEN}cd $MULTIPLEXOR_DIR && npm start${NC}"
    echo ""
fi
