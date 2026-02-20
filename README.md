# UAV System Identification & Simulation Tooling

This repository contains tools and models for a MATLAB-based physics backend used with ArduPilot SITL.

## Repository structure

### MATLAB_SITL_physics_backend/
This folder contains the MATLAB/Simulink physics backend.
Models are defined in MATLAB and Simulink and represent the UAV flight physics that SITL uses as an external simulation backend.

### ehicle_simulation/
This folder contains the automated SITL workflow that starts a full simulation stack:
- ArduPilot SITL is started inside WSL
- SITL is configured to use the MATLAB physics backend
- A Ground Control Station (Mission Planner) is launched
You can then control and fly the virtual UAV through Mission Planner while the dynamics are simulated by the MATLAB/Simulink backend.

## Quick start

1. Ensure WSL + ArduPilot SITL are installed and runnable.
2. Ensure MATLAB/Simulink is installed and the required toolboxes/add-ons are available.
3. Use the scripts in ehicle_simulation/ to launch the SITL + MATLAB backend + Mission Planner workflow.

## Notes
- Large generated build artifacts (e.g. slprj/) and log files should typically not be committed.
