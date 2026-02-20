function sim_service(cmd)
% SIM_SERVICE  Start/Stop/Pause/Continue für Copter-Modell
%
% Aufruf:
%   sim_service("start")
%   sim_service("stop")
%   sim_service("pause")
%   sim_service("continue")
%   sim_service("status")

model = "Copter";

% Pfade für Physics Backend und alle Unterordner setzen, damit alle
% benötigten Funktionen, Bibliotheken (z.B. AP_Conector_new, pnet-MEX) usw.
% gefunden werden.
thisFile = mfilename("fullpath");
thisDir  = fileparts(thisFile);                     % .../Copter/simulink_models
copterDir = fileparts(thisDir);                     % .../Copter
backendRoot = fileparts(copterDir);                 % .../MATLAB_SITL_physics_backend

% Kompletten Backend-Baum nach vorne auf den Pfad setzen
addpath(genpath(backendRoot));

% Versuche, die AP-Connector-Library vorzuladen (falls vorhanden)
try
    if ~bdIsLoaded("AP_Conector_new")
        load_system("AP_Conector_new");
    end
catch ME
    disp("Warnung: Konnte AP_Conector_new nicht laden:");
    disp(getReport(ME));
end

% Modell laden falls nicht geladen
if ~bdIsLoaded(model)
    load_system(model);
end

status = get_param(model,"SimulationStatus");

switch lower(cmd)

    case "start"
        if status == "stopped"
            set_param(model,"StopTime","inf");
            set_param(model,"SimulationCommand","start");
            disp("Simulation gestartet (inf)")
        else
            disp("Simulation läuft bereits oder ist pausiert")
        end

    case "stop"
        if status ~= "stopped"
            set_param(model,"SimulationCommand","stop");
            disp("Simulation gestoppt")
        else
            disp("Simulation ist bereits gestoppt")
        end

    case "pause"
        if status == "running"
            set_param(model,"SimulationCommand","pause");
            disp("Simulation pausiert")
        else
            disp("Pause nicht möglich (nicht running)")
        end

    case "continue"
        if status == "paused"
            set_param(model,"SimulationCommand","continue");
            disp("Simulation fortgesetzt")
        else
            disp("Continue nicht möglich (nicht paused)")
        end

    case "status"
        disp("Status: " + status)

    otherwise
        error("Unbekannter Befehl: start | stop | pause | continue | status")
end
end