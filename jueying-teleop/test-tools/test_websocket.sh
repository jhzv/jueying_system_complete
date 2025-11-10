#!/bin/bash

echo "🔌 Probando WebSocket de Rosbridge"
echo "===================================="

# Usar Python para probar WebSocket (más confiable que curl)
python3 << 'PYTHON_SCRIPT'
import socket
import sys

def test_websocket():
    try:
        # Crear socket TCP
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        
        # Conectar al puerto 9090
        sock.connect(('localhost', 9090))
        
        # Enviar handshake WebSocket
        handshake = (
            "GET / HTTP/1.1\r\n"
            "Host: localhost:9090\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "\r\n"
        )
        
        sock.sendall(handshake.encode())
        
        # Recibir respuesta
        response = sock.recv(1024).decode()
        
        if "101" in response and "Switching Protocols" in response:
            print("✅ WebSocket handshake exitoso!")
            print("✅ Rosbridge acepta conexiones WebSocket")
            return True
        else:
            print("❌ Respuesta inesperada:")
            print(response[:200])
            return False
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False
    finally:
        sock.close()

result = test_websocket()
sys.exit(0 if result else 1)

PYTHON_SCRIPT

