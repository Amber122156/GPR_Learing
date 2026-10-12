function output = flappy_sil_controller(sensor)
%FLAPPY_SIL_CONTROLLER 500 Hz deterministic attitude and altitude controller.
% Input: [time; position(3); velocity(3); quaternion(wxyz); omega(3); normal].
% Output: [frequency; torque(3); z_ref; z_error; attitude_error(3); liftoff].

persistent liftoffTime liftoffZ
cfg = flappy_sil_active_config();
sensor = sensor(:);
t = sensor(1);
position = sensor(2:4);
velocity = sensor(5:7);
quaternion = sensor(8:11);
omega = sensor(12:14);
normalForce = sensor(15);
if isempty(liftoffTime) || t <= 1e-12
    liftoffTime = inf;
    liftoffZ = cfg.initial_torso_z_m;
end

if ~all(isfinite(quaternion)) || norm(quaternion) < 1e-9
    quaternion = [1;0;0;0];
else
    quaternion = quaternion/norm(quaternion);
end
if quaternion(1) < 0, quaternion = -quaternion; end
if ~all(isfinite(omega)), omega = zeros(3,1); end

phase = min(max(t/cfg.takeoff_ramp_s,0),1);
takeoffFrequency = cfg.takeoff_frequency_hz*(3*phase^2-2*phase^3);
if isinf(liftoffTime) && t >= cfg.takeoff_ramp_s && ...
        normalForce < cfg.liftoff_normal_threshold_N
    liftoffTime = t;
    liftoffZ = position(3);
end
isHover = ~isinf(liftoffTime);
referenceZ = liftoffZ;
frequencyCommand = takeoffFrequency;
if isHover
    referencePhase = min(max((t-liftoffTime)/cfg.reference_ramp_s,0),1);
    referenceZ = liftoffZ + (cfg.hover_height_m-liftoffZ)* ...
        (3*referencePhase^2-2*referencePhase^3);
    zError = referenceZ-position(3);
    [roll,pitch,~] = flappy_quaternion_to_euler(quaternion.');
    [frequencyCommand,heightDiagnostic] = flappy_sil_flappy_height_pid( ...
        referenceZ,position(3),velocity(3),roll,pitch,true,t,cfg);
else
    zError = referenceZ-position(3);
    [~,heightDiagnostic] = flappy_sil_flappy_height_pid( ...
        referenceZ,position(3),velocity(3),0,0,false,t,cfg);
end

attitudeError = flappy_quaternion_attitude_error(quaternion,[1;0;0;0]);
trimTorque = flappy_sil_attitude_trim(attitudeError,isHover,t);
torqueCommand = -cfg.attitude_kp_Nm_rad(:).*attitudeError - ...
    cfg.attitude_kd_Nms_rad(:).*omega + trimTorque;
torqueCommand = cfg.virtual_torque_axis_sign(:).*torqueCommand;
% Final four fields are reserved for position-loop diagnostics so all SIL
% variants have a stable 14-element controller logging interface.
output = [frequencyCommand; torqueCommand; referenceZ; zError; attitudeError; ...
    double(isHover); 0; 0; 0; 0; 0; 0; trimTorque; heightDiagnostic];
end
