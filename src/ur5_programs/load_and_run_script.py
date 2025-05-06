#!/usr/bin/env python3
import socket
import time
import sys

# === CONFIGURATION ===
ROBOT_IP        = "192.168.1.103"
DASHBOARD_PORT  = 29999             # Dashboard server port
PROGRAM_NAME    = "ur5_control.urp"     # Name of the URP already on the robot
CMD_TIMEOUT     = 3.0               # seconds to wait for dashboard responses

def send_dashboard(cmd: str) -> str:
    """
    Send one command to the Dashboard server and return its response.
    """
    with socket.create_connection((ROBOT_IP, DASHBOARD_PORT), timeout=CMD_TIMEOUT) as sock:
        # read and discard the welcome banner
        sock.recv(1024)
        sock.sendall((cmd + "\n").encode("utf-8"))
        time.sleep(0.1)
        return sock.recv(4096).decode("utf-8", errors="ignore").strip()

def load_program(name: str):
    print(f"→ DASHBOARD: load {name}")
    resp = send_dashboard(f"load {name}")
    print("   " + resp)
    if "Loading program" not in resp:
        raise RuntimeError(f"Failed to load program: {resp}")

def play_program():
    print("→ DASHBOARD: play")
    resp = send_dashboard("play")
    print("   " + resp)
    if not any(k in resp for k in ("Starting program", "Running program")):
        raise RuntimeError(f"Failed to play program: {resp}")

def main():
    try:
        load_program(PROGRAM_NAME)
        play_program()
        print("✔ Program is now running on the robot.")
    except Exception as e:
        print(f"⚠ Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        PROGRAM_NAME = sys.argv[1]
    main()

