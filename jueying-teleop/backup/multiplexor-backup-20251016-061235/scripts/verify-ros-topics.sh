#!/bin/bash
# scripts/verify-ros-topics.sh
# Verifica que todos los tópicos ROS necesarios estén disponibles

echo "======================================"
echo "Verificación de Tópicos ROS - Jueying"
echo "======================================"
echo ""

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Verificar que roscore está corriendo
echo "Verificando roscore..."
if ! pgrep -x "roscore" > /dev/null; then
    echo -e "${RED}✗ roscore no está corriendo${NC}"
    echo "  Por favor iniciar roscore primero: roscore &"
    exit 1
fi
echo -e "${GREEN}✓ roscore está corriendo${NC}"
echo ""

# Función para verificar si un tópico existe
check_topic() {
    local topic=$1
    local description=$2
    
    if rostopic list 2>/dev/null | grep -q "^${topic}$"; then
        echo -e "${GREEN}✓${NC} $topic - $description"
        return 0
    else
        echo -e "${RED}✗${NC} $topic - $description ${RED}(NO ENCONTRADO)${NC}"
        return 1
    fi
}

echo "Tópicos de COMANDOS (debe publicar el multiplexor):"
echo "------------------------------------------------"
check_topic "/cmd_vel" "Control de velocidades"
check_topic "/simple_cmd" "Comandos discretos"
check_topic "/complex_cmd" "Comandos con parámetros"
echo ""

echo "Tópicos de FEEDBACK (debe recibir el multiplexor):"
echo "------------------------------------------------"
check_topic "/leg_odom" "Odometría de patas"
check_topic "/imu/data" "Datos de IMU"
check_topic "/joint_states" "Estados de articulaciones"
check_topic "/handle_state" "Estado del handle"
echo ""

# Verificar nodos críticos
echo "Nodos ROS Críticos:"
echo "------------------------------------------------"

check_node() {
    local node=$1
    local description=$2
    
    if rosnode list 2>/dev/null | grep -q "$node"; then
        echo -e "${GREEN}✓${NC} $node - $description"
        return 0
    else
        echo -e "${YELLOW}⚠${NC} $node - $description ${YELLOW}(NO ENCONTRADO)${NC}"
        return 1
    fi
}

check_node "qnx2ros" "Recibe datos del robot"
check_node "ros2qnx" "Envía comandos al robot"
check_node "rosbridge" "Bridge WebSocket"
echo ""

# Verificar rosbridge específicamente
echo "Verificando rosbridge_server:"
echo "------------------------------------------------"
if netstat -tlnp 2>/dev/null | grep -q ":9090 "; then
    echo -e "${GREEN}✓ rosbridge escuchando en puerto 9090${NC}"
else
    echo -e "${RED}✗ rosbridge NO está escuchando en puerto 9090${NC}"
    echo "  Iniciar con: roslaunch rosbridge_server rosbridge_websocket.launch"
fi
echo ""

# Test de publicación (opcional)
echo "Test de Publicación (opcional):"
echo "------------------------------------------------"
read -p "¿Desea probar publicación en /simple_cmd? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Publicando comando 'stand' a /simple_cmd..."
    rostopic pub -1 /simple_cmd message_transformer/SimpleCMD "data: 'stand'" 2>/dev/null
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Publicación exitosa${NC}"
    else
        echo -e "${RED}✗ Error en publicación${NC}"
        echo "  Verificar que el mensaje message_transformer/SimpleCMD esté definido"
    fi
fi
echo ""

# Test de suscripción (opcional)
echo "Test de Suscripción (opcional):"
echo "------------------------------------------------"
read -p "¿Desea verificar mensajes en /leg_odom? (esperará 5 segundos) (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Escuchando /leg_odom por 5 segundos..."
    timeout 5 rostopic echo /leg_odom -n 1 2>/dev/null
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Recibiendo datos de odometría${NC}"
    else
        echo -e "${YELLOW}⚠ No se recibieron datos de odometría${NC}"
        echo "  Verificar que el robot esté encendido y qnx2ros esté publicando"
    fi
fi
echo ""

# Resumen
echo "======================================"
echo "Resumen"
echo "======================================"

# Contar tópicos presentes
topics_present=0
topics_total=7

for topic in "/cmd_vel" "/simple_cmd" "/complex_cmd" "/leg_odom" "/imu/data" "/joint_states" "/handle_state"; do
    if rostopic list 2>/dev/null | grep -q "^${topic}$"; then
        ((topics_present++))
    fi
done

echo "Tópicos encontrados: $topics_present/$topics_total"

if [ $topics_present -eq $topics_total ]; then
    echo -e "${GREEN}✓ Sistema listo para usar con el multiplexor${NC}"
    exit 0
elif [ $topics_present -ge 3 ]; then
    echo -e "${YELLOW}⚠ Sistema parcialmente listo${NC}"
    echo "  Algunos tópicos faltan. Verificar que todos los nodos estén corriendo."
    exit 0
else
    echo -e "${RED}✗ Sistema NO está listo${NC}"
    echo "  Muchos tópicos faltan. Verificar instalación de ROS y nodos del robot."
    exit 1
fi
