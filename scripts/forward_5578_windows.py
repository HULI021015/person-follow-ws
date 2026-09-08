"""Run on Windows: 127.0.0.1:5578 <-> CAN115 192.168.0.7:5578."""

import socket
import threading

LISTEN_HOST = "127.0.0.1"
LISTEN_PORT = 5578
TARGET_HOST = "192.168.0.7"
TARGET_PORT = 5578


def relay(source, destination):
    try:
        while data := source.recv(4096):
            destination.sendall(data)
    except OSError:
        pass
    finally:
        source.close()
        destination.close()


def handle(client):
    try:
        target = socket.create_connection((TARGET_HOST, TARGET_PORT), timeout=5)
    except OSError as exc:
        print(f"CAN115 connection failed: {exc}", flush=True)
        client.close()
        return
    threading.Thread(target=relay, args=(client, target), daemon=True).start()
    threading.Thread(target=relay, args=(target, client), daemon=True).start()


with socket.create_server((LISTEN_HOST, LISTEN_PORT), reuse_port=False) as server:
    print(f"relay {LISTEN_HOST}:{LISTEN_PORT} -> {TARGET_HOST}:{TARGET_PORT}", flush=True)
    while True:
        connection, _ = server.accept()
        threading.Thread(target=handle, args=(connection,), daemon=True).start()

