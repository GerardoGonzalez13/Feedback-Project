import json
import socket
import time

WINDOWS_HOST_IP = "172.26.96.1"   # Windows<->WSL gateway
CMD_PORT = 49006

def send(sock, msg):
    sock.sendto(json.dumps(msg).encode("utf-8"), (WINDOWS_HOST_IP, CMD_PORT))

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # enable control and roll right gently for 2 seconds
    send(sock, {"enable": 1, "roll": 0.2, "pitch": 0.0, "yaw": 0.0})
    time.sleep(2.0)

    # neutral for 2 seconds
    send(sock, {"enable": 1, "roll": 0.0, "pitch": 0.0, "yaw": 0.0})
    time.sleep(2.0)

    # release control back to user
    send(sock, {"enable": 0})
    sock.close()

if __name__ == "__main__":
    main()