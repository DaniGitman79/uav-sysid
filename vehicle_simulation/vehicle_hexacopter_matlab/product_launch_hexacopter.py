#!/usr/bin/env python3
"""
Vehicle Launcher – Lifecycle-Gerüst + config.json

Zustände:
STARTING -> RUNNING -> SHUTTING_DOWN -> EXIT

- launcher.py und config.json liegen im gleichen Ordner
- keine Shutdown-Logik implementiert (nur Gerüst/Platzhalter)
"""

import os
import json
import subprocess
import signal
import sys
import time

STATE = "INIT"

# Prozess-Handles (nur für Vollständigkeit/Debug; Shutdown ist hier noch Platzhalter)
p_sitl = None
p_matlab = None
p_mp = None


def load_config():
    """Lädt config.json aus dem gleichen Ordner wie diese Python-Datei."""
    here = os.path.dirname(os.path.abspath(__file__))
    cfg_path = os.path.join(here, "config.json")
    if not os.path.exists(cfg_path):
        raise FileNotFoundError(f"config.json not found next to launcher: {cfg_path}")

    with open(cfg_path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def start_services(cfg: dict):
    """STARTING: startet SITL, MATLAB, Mission Planner basierend auf config.json."""
    global p_sitl, p_matlab, p_mp

    print("[STATE] STARTING")

    # --- 1) SITL ---
    sitl = cfg.get("sitl", {})
    use_wsl = sitl.get("use_wsl", True)
    wsl_shell = sitl.get("wsl_shell", "bash -lc")  # e.g. "bash -lc"
    command = sitl.get("command", "sim_vehicle.py")
    vehicle_type = sitl.get("vehicle_type", "ArduCopter")
    model = sitl.get("model", "JSON")
    model_arg = sitl.get("model_arg", "172.28.176.1")
    extra_args = sitl.get("extra_args", "")

    sitl_cmd = f"{command} -v {vehicle_type} --model {model}:{model_arg} {extra_args}".strip()
    print("[START] SITL:", sitl_cmd)


    shell_parts = wsl_shell.split()  # ["bash","-lc"]
    p_sitl = subprocess.Popen(["wsl", *shell_parts, sitl_cmd])
    

    # --- 2) MATLAB ---
    matlab = cfg.get("matlab", {})
    matlab_exe = matlab.get("exe", r"C:\Program Files\MATLAB\R2025a\bin\matlab.exe")
    matlab_script = matlab.get("script", "MATLAB_SITL_physics_backend/Copter/SIM_multicopter.m")
    nosplash = matlab.get("nosplash", True)
    nodesktop = matlab.get("nodesktop", True)

    # Script-Pfad relativ zum Repo-Root: wir nehmen "launcher-ordner/.. /.." als BASE
    here = os.path.dirname(os.path.abspath(__file__))
    base = os.path.abspath(os.path.join(here, "..", ".."))

    matlab_script_abs = os.path.join(base, matlab_script).replace("\\", "/")
    flags = []
    if nosplash:
        flags.append("-nosplash")
    if nodesktop:
        flags.append("-nodesktop")
    flags_str = " ".join(flags)

    matlab_cmd = f'"{matlab_exe}" {flags_str} -r "run(\'{matlab_script_abs}\');"'
    print("[START] MATLAB:", matlab_cmd)

    p_matlab = subprocess.Popen(matlab_cmd, shell=True)

    # --- 3) Mission Planner ---
    mp = cfg.get("mission_planner", {})
    mp_exe = mp.get("exe", r"C:\Program Files (x86)\Mission Planner\MissionPlanner.exe")
    print("[START] Mission Planner:", mp_exe)

    p_mp = subprocess.Popen([mp_exe])


def run_loop():
    """RUNNING: hält den Parent-Prozess am Leben."""
    print("[STATE] RUNNING")
    print("Press Ctrl+C to stop.")
    while True:
        time.sleep(1)


def shutdown_services():
    """SHUTTING_DOWN: Platzhalter (noch keine Stop-Logik)."""
    print("[STATE] SHUTTING_DOWN")
    # Hier später Stop-Logik:
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

    cfg = load_config()

    STATE = "STARTING"
    start_services(cfg)

    STATE = "RUNNING"
    run_loop()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, handle_interrupt)
    main()
