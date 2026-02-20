# Workflow: MATLAB/Simulink Physics Backend + ArduPilot SITL (WSL) + Mission Planner (Windows)

## Prerequisites
- Windows + WSL2 (Ubuntu)
- ArduPilot installed in WSL via: `bash scripts/bootstrap_wsl_ardupilot.sh`
- MATLAB/Simulink on Windows
- Mission Planner on Windows

## Ports
Configured in `config/sitl/ports.env`:
- Mission Planner listens on UDP 14550
- Optional second MAVLink out on UDP 14560
- JSON sim port reserved: 9002

## Run order
1) Start MATLAB physics backend:
   - MATLAB: `run("src/matlab/scripts/start_physics_backend.m")`

2) Start SITL in WSL from repo root:
   - WSL: `bash src/wsl/sitl/start_sitl_json.sh`

3) Open Mission Planner:
   - Connect to UDP port 14550 (localhost)

## Notes
- `start_sitl_json.sh` forces SITL MAVLink output to Mission Planner via `--out udp:127.0.0.1:14550`.
- The exact JSON simulator hook depends on your current ArduPilot command line; adapt the "USER ADAPT" section accordingly.
