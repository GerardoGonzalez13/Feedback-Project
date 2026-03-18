import json
import socket
import time

WINDOWS_HOST_IP = "172.26.96.1"
CMD_PORT = 49006

RATE_HZ = 25          # command send rate (keep < plugin HZ)
DT = 1.0 / RATE_HZ

def send(sock, msg):
    sock.sendto(json.dumps(msg).encode("utf-8"), (WINDOWS_HOST_IP, CMD_PORT))

def hold(sock, duration_s, roll=0.0, pitch=0.0, yaw=0.0, enable=1):
    """Send the same command repeatedly for duration_s."""
    t_end = time.time() + duration_s
    while time.time() < t_end:
        send(sock, {"enable": enable, "roll": roll, "pitch": pitch, "yaw": yaw})
        time.sleep(DT)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 0) Take control and settle at neutral (override ON)
    hold(sock, 1.0, enable=1, roll=0.0, pitch=0.0, yaw=0.0)

    # 1) Roll-in (open-loop first pass)
    # Tune these numbers to hit ~60 deg bank on the King Air
    hold(sock, 3.0, enable=1, roll=0.35, pitch=0.0, yaw=0.0)

    # 2) Hold-ish near bank angle
    hold(sock, 2.0, enable=1, roll=0.10, pitch=0.0, yaw=0.0)

    # 3) Rudder pulse while holding bank
    hold(sock, 0.4, enable=1, roll=0.10, pitch=0.0, yaw=0.20)
    hold(sock, 0.6, enable=1, roll=0.10, pitch=0.0, yaw=0.00)

    # 4) "Hands-off" baseline window: NEUTRAL COMMANDS but still enabled
    hold(sock, 20.0, enable=1, roll=0.0, pitch=0.0, yaw=0.0)

    # 5) End of run: release control back to mouse/user
    send(sock, {"enable": 0})
    sock.close()

if __name__ == "__main__":
    main()