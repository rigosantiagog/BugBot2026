import socket
from datetime import datetime

HOST = "0.0.0.0"
PORT = 5000
LOG_FILE = "robot_anillo.log"

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

server.bind((HOST, PORT))
server.listen(1)

print(f"Servidor escuchando en puerto {PORT}")

while True:
    cliente, addr = server.accept()

    print(f"\n[+] ESP32 conectado desde {addr}")

    try:
        with open(LOG_FILE, "a", encoding="utf-8") as log:
            while True:

                data = cliente.recv(1024)

                if not data:
                    print("[-] ESP32 desconectado")
                    break

                texto = data.decode("utf-8", errors="replace").strip()

                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                linea = f"[{timestamp}] {texto}"

                # Mostrar en terminal
                print(linea)

                # Guardar en archivo
                log.write(linea + "\n")
                log.flush()

    except Exception as e:
        print("Error:", e)

    cliente.close()
