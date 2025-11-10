#!/usr/bin/env python3
"""
Script de prueba simple para rosbridge
"""
import asyncio
import websockets
import json
import sys

ROSBRIDGE_URL = "ws://localhost:9090"

async def test_rosbridge():
    print("🔍 Probando conexión a rosbridge...")
    print(f"URL: {ROSBRIDGE_URL}")
    
    try:
        async with websockets.connect(ROSBRIDGE_URL) as websocket:
            print("✅ Conectado a rosbridge!")
            
            # 1. Subscribirse a /rosout
            subscribe_msg = {
                "op": "subscribe",
                "topic": "/rosout"
            }
            await websocket.send(json.dumps(subscribe_msg))
            print("📡 Subscrito a /rosout")
            
            # 2. Recibir algunos mensajes
            print("\n🎧 Escuchando mensajes (5 segundos)...\n")
            try:
                for i in range(5):
                    message = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                    data = json.loads(message)
                    if 'topic' in data:
                        print(f"📨 Mensaje recibido de: {data['topic']}")
                    else:
                        print(f"📨 Mensaje: {data.get('op', 'unknown')}")
            except asyncio.TimeoutError:
                print("⏱️  Timeout esperando mensajes")
            
            # 3. Publicar un mensaje de prueba
            print("\n📤 Publicando mensaje de prueba a /test_topic...")
            publish_msg = {
                "op": "publish",
                "topic": "/test_topic",
                "msg": {
                    "data": "Hola desde script de prueba!"
                }
            }
            await websocket.send(json.dumps(publish_msg))
            print("✅ Mensaje publicado")
            
            # 4. Llamar a un servicio (rostopic list)
            print("\n📋 Obteniendo lista de tópicos...")
            topics_msg = {
                "op": "call_service",
                "service": "/rosapi/topics"
            }
            await websocket.send(json.dumps(topics_msg))
            
            response = await asyncio.wait_for(websocket.recv(), timeout=3.0)
            topics_data = json.loads(response)
            if 'values' in topics_data:
                print(f"✅ Tópicos disponibles: {len(topics_data['values']['topics'])} encontrados")
                print("Algunos tópicos:")
                for topic in topics_data['values']['topics'][:5]:
                    print(f"  - {topic}")
            
            print("\n✅ TODAS LAS PRUEBAS EXITOSAS")
            return True
            
    except ConnectionRefusedError:
        print("❌ Error: No se pudo conectar a rosbridge")
        print("   Verifica que rosbridge esté corriendo:")
        print("   $ roslaunch rosbridge_server rosbridge_websocket.launch")
        return False
    except Exception as e:
        print(f"❌ Error inesperado: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    try:
        result = asyncio.run(test_rosbridge())
        sys.exit(0 if result else 1)
    except KeyboardInterrupt:
        print("\n⚠️  Prueba interrumpida por usuario")
        sys.exit(1)
