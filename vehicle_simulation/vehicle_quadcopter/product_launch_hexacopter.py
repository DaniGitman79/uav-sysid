#!/usr/bin/env python3
"""
Hexacopter Launcher – Lifecycle-Gerüst

Zustände:
STARTING -> RUNNING -> SHUTTING_DOWN -> EXIT
"""

import os
import subprocess
import signal
import sys
import time

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

STATE = "INIT"

# Prozess-Handles
p_sitl = None
p_matlab = None
p_mp = None


def start_services():
    global p_sitl, p_matlab, p_mp

    print("[STATE] STARTING")

    # 1) SITL
    SITL_CMD = "sim_vehicle.py -v ArduCopter --model JSON:172.28.176.1"
    p_sitl = subprocess.Popen(["wsl", "bash", "-lc", SITL_CMD])

    # 2) MATLAB
    MATLAB_EXE = r"C:\Program Files\MATLAB\R2025a\bin\matlab.exe"
    MATLAB_SCRIPT = os.path.join(
        BASE,
        "MATLAB_SITL_physics_backend",
        "Copter",
        "SIM_multicopter.m"
    ).replace("\\", "/")

    MATLAB_CMD = (
        f'"{MATLAB_EXE}" -nosplash -nodesktop '
        f'-r "run(\'{MATLAB_SCRIPT}\');"'
    )

    p_matlab = subprocess.Popen(MATLAB_CMD, shell=True)

    # 3) Mission Planner
    MISSION_PLANNER_EXE = r"C:\Program Files (x86)\Mission Planner\MissionPlanner.exe"
    p_mp = subprocess.Popen([MISSION_PLANNER_EXE])


def run_loop():
    print("[STATE] RUNNING")
    print("Press Ctrl+C to stop.")
    while True:
        time.sleep(1)


def shutdown_services():
    print("[STATE] SHUTTING_DOWN")

    # Hier kommt später deine Stop-Logik rein:
    # - MATLAB terminate
    # - Mission Planner taskkill
    # - WSL shutdown mit Timeout

    print("[STATE] EXIT")


def handle_interrupt(sig, frame):
    global STATE
    STATE = "SHUTTING_DOWN"
    shutdown_services()
    sys.exit(0)


def main():
    global STATE

    STATE = "STARTING"
    start_services()

    STATE = "RUNNING"
    run_loop()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, handle_interrupt)
    main()
