function [pwm, reset] = AP_receve_new(time)
% AP_receve  Receive PWM + frame timing from ArduPilot SITL via UDP
% - Stores dt derived from SITL frame_rate in global AP_last_dt
% - Keeps original behavior (blocking read, frame checks, fps print)

global u
global AP_last_dt
persistent connected frame_time last_sim_time frame_count last_SITL_frame past_time

% Init / reset on first call or Simulink time reset
if isempty(u) || time == 0
    try
        pnet('closeall');
    catch
    end

    u = pnet('udpsocket', 9002);
    pnet(u,'setwritetimeout', 1);
    pnet(u,'setreadtimeout', 0);

    connected       = false;
    frame_time      = tic;
    last_sim_time   = 0;
    frame_count     = 0;
    last_SITL_frame = -1;
    past_time       = -1;

    AP_last_dt = [];  % dt unknown until first valid packet
end

% Protect against repeated Simulink time values (debug)
if past_time == time
    error('AP_receve: Receive repeat time');
end
past_time = time;

print_frame_count = 1000;
bytes_read = 4 + 4 + 16*2; % UINT16 magic + UINT16 frame_rate + UINT32 frame + 16*UINT16 pwm
reset = false;
pwm = zeros(16,1);

% Wait for one complete packet
while true
    in_bytes = pnet(u,'readpacket', bytes_read);
    if in_bytes <= 0
        continue;
    end

    % If buffer contains more than one frame, drop older ones by looping
    if in_bytes > bytes_read
        if in_bytes == u.InputBufferSize
            fprintf('Buffer reset\n');
        end
        continue;
    end

    % --- Parse packet ---
    magic = pnet(u,'read', 1, 'UINT16', 'intel');
    frame_rate = double(pnet(u,'read', 1, 'UINT16', 'intel'));
    SITL_frame = pnet(u,'read', 1, 'UINT32', 'intel');
    pwm = double(pnet(u,'read', 16, 'UINT16', 'intel'))';

    % Validate
    if magic ~= 18458
        warning('AP_receve: incorrect magic value (%d)', magic);
        continue;
    end

    % Store dt from SITL
    if isfinite(frame_rate) && frame_rate > 0
        AP_last_dt = 1 / frame_rate;
    else
        warning('AP_receve: invalid frame_rate=%g (keeping previous dt)', frame_rate);
    end

    % Frame ordering checks
    if SITL_frame < last_SITL_frame
        connected = false;
        reset = true;
        fprintf('Controller reset\n');
    elseif SITL_frame == last_SITL_frame
        fprintf('Duplicate input frame\n');
        continue;
    elseif SITL_frame ~= last_SITL_frame + 1 && connected
        fprintf('Missed %i input frames\n', SITL_frame - last_SITL_frame - 1);
    end
    last_SITL_frame = SITL_frame;

    break;
end

% Print connection info once
if ~connected
    connected = true;
    [ip, port] = pnet(u,'gethost');
    fprintf('Connected to %i.%i.%i.%i:%i\n', ip, port);
end

% FPS / realtime ratio print
frame_count = frame_count + 1;
if rem(frame_count, print_frame_count) == 0
    total_time = toc(frame_time);
    frame_time = tic;
    sim_time = time - last_sim_time;
    last_sim_time = time;
    time_ratio = sim_time / total_time;
    fprintf("%0.2f fps, %0.2f%% of realtime\n", print_frame_count/total_time, time_ratio*100);

    if ~isempty(AP_last_dt)
        fprintf("AP_last_dt=%g (rate=%0.1f Hz)\n", AP_last_dt, 1/AP_last_dt);
    end
end
end
