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
p_matlab_engine = None


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
    matlab_exe = matlab.get("exe", r"C\\Program Files\\MATLAB\\R2025a\\bin\\matlab.exe")
    # config 'script' should point to the sim_service.m location (file or folder)
    matlab_script = matlab.get("script", "MATLAB_SITL_physics_backend/Copter/simulink_models/sim_service.m")
    nosplash = matlab.get("nosplash", True)
    nodesktop = matlab.get("nodesktop", True)

    # Script-Pfad relativ zum Repo-Root
    here = os.path.dirname(os.path.abspath(__file__))
    base = os.path.abspath(os.path.join(here, "..", ".."))
    matlab_script_abs = os.path.abspath(os.path.join(base, matlab_script))

    # Allow "script" to be either a file (sim_service.m) or a directory
    if os.path.isdir(matlab_script_abs):
        service_dir = matlab_script_abs
    else:
        service_dir = os.path.dirname(matlab_script_abs)

    # Normalize to a form MATLAB can handle
    service_dir = service_dir.replace("\\", "/")

    print("[MATLAB] Using service_dir:", service_dir)

    # Start MATLAB as a persistent process and call sim_service('start').
    # We explicitly change into the folder that contains sim_service.m so that
    # MATLAB can always find the function, even wenn addpath fehlschlägt.
    matlab_args = []
    if nosplash:
        matlab_args.append("-nosplash")
    if nodesktop:
        matlab_args.append("-nodesktop")

    # Important: do NOT use -batch here, because -batch exits MATLAB after executing
    # the command. We want MATLAB/Simulink to stay alive as physics backend.
    code = (
        f"cd('{service_dir}'); "
        f"addpath('{service_dir}'); "
        f"try, sim_service('start'); catch e, disp(getReport(e)), end"
    )

    cmd_list = [matlab_exe, *matlab_args, "-r", code]
    print("[START] MATLAB (persistent -r):", cmd_list)
    p_matlab = subprocess.Popen(cmd_list)

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
    # Stop MATLAB cleanly if we used the MATLAB Engine
    global p_matlab_engine, p_matlab, p_mp, p_sitl
    if p_matlab_engine is not None:
        try:
            print("[MATLAB] Calling sim_service('stop') via engine")
            p_matlab_engine.sim_service('stop', nargout=0)
            p_matlab_engine.quit()
            p_matlab_engine = None
            print("[MATLAB] Engine session stopped")
        except Exception as e:
            print("[MATLAB] Failed to stop engine session:", e)

    else:
        # Fallback: if we started MATLAB as subprocess, attempt graceful stop
        if p_matlab is not None:
            try:
                print("[MATLAB] Attempting graceful stop via short MATLAB call")
                matlab = load_config().get('matlab', {})
                matlab_exe = matlab.get('exe', r"C:\Program Files\MATLAB\R2025a\bin\matlab.exe")
                # use the same script location from config to build path
                matlab_script = matlab.get('script', 'MATLAB_SITL_physics_backend/Copter/simulink_models/sim_service.m')
                here = os.path.dirname(os.path.abspath(__file__))
                base = os.path.abspath(os.path.join(here, "..", ".."))
                matlab_script_abs = os.path.abspath(os.path.join(base, matlab_script)).replace('\\', '/')
                service_dir = os.path.dirname(matlab_script_abs).replace('\\', '/')
                # Use -batch to call sim_service('stop') and exit cleanly
                code = f"addpath('{service_dir}'); try, sim_service('stop'); catch e, disp(getReport(e)), end"
                cmd_list = [matlab_exe, "-nosplash", "-nodesktop", "-batch", code]
                subprocess.Popen(cmd_list)
                time.sleep(1)
            except Exception as e:
                print("[MATLAB] Graceful stop failed, terminating process:", e)
            try:
                p_matlab.terminate()
            except Exception:
                pass

    # TODO: Mission Planner / SITL cleanup (best-effort)
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
