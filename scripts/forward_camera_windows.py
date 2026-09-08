"""Windows TCP relay for WSL NAT: WSL gateway:37778 -> Dahua:37777."""

from __future__ import annotations

import argparse
import socket
import threading


def relay(source: socket.socket, destination: socket.socket) -> None:
    try:
        while data := source.recv(65536):
            destination.sendall(data)
    except OSError:
        pass
    finally:
        try:
            source.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            destination.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        source.close()
        destination.close()


def handle(client: socket.socket, target_host: str, target_port: int) -> None:
    try:
        target = socket.create_connection((target_host, target_port), timeout=5)
        target.settimeout(None)
    except OSError as exc:
        print(f"Dahua connection failed: {exc}", flush=True)
        client.close()
        return
    threading.Thread(target=relay, args=(client, target), daemon=True).start()
    threading.Thread(target=relay, args=(target, client), daemon=True).start()


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--listen-host",
        required=True,
        help="WSL gateway address from: grep nameserver /etc/resolv.conf",
    )
    parser.add_argument("--listen-port", type=int, default=37778)
    parser.add_argument("--target-host", default="192.168.1.21")
    parser.add_argument("--target-port", type=int, default=37777)
    return parser.parse_args()


def main() -> None:
    args = arguments()
    with socket.create_server((args.listen_host, args.listen_port), reuse_port=False) as server:
        print(
            f"relay {args.listen_host}:{args.listen_port} -> "
            f"{args.target_host}:{args.target_port}",
            flush=True,
        )
        while True:
            connection, _address = server.accept()
            threading.Thread(
                target=handle,
                args=(connection, args.target_host, args.target_port),
                daemon=True,
            ).start()


if __name__ == "__main__":
    main()
