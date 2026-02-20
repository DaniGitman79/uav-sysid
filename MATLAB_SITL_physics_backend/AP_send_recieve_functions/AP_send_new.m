function AP_send_new(gyro, attitude, accel, velocity, position, time)
global u
global AP_last_dt   % <-- kommt aus AP_receve

if isempty(u)
   return
end

% frame-synchrone Zeit (wie physics_time_s im MATLAB-Backend)
persistent sim_time past_time
if isempty(sim_time)
    sim_time = 0;
end
if isempty(past_time)
    past_time = -1;
end

% optional: Simulink-time Duplikate weiter prüfen (Debug)
if past_time == time
    error('Send repeat time');
end
past_time = time;

% monotone Zeit auf Basis der SITL frame_rate
if ~isempty(AP_last_dt) && isfinite(AP_last_dt) && AP_last_dt > 0
    sim_time = sim_time + AP_last_dt;
else
    % falls dt noch nicht gesetzt wurde, nutze notfalls Simulink time
    % (sollte nach dem ersten Receive nicht mehr passieren)
    sim_time = time;
end

% build structure representing the JSON string to be sent
JSON.timestamp = sim_time;
JSON.imu.gyro = gyro;
JSON.imu.accel_body = accel;
JSON.position = position;
JSON.attitude = attitude;
JSON.velocity = velocity;

% Report to AP
pnet(u,'printf',sprintf('\n%s\n',jsonencode(JSON)));
pnet(u,'writepacket');
end
