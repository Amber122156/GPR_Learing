function output = flappy_sil_position_controller(sensor)
%FLAPPY_SIL_POSITION_CONTROLLER Cascaded XY position, attitude and height SIL.
% Output adds [a_xy_command(2); xy_reference(2)] after the shared 10 fields.

persistent liftoffTime liftoffPosition referenceYaw accelerationXY lastUpdate
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
    liftoffPosition = [0;0;cfg.initial_torso_z_m];
    referenceYaw = 0;
    accelerationXY = zeros(2,1);
    lastUpdate = -inf;
end
if ~all(isfinite(quaternion)) || norm(quaternion) < 1e-9
    quaternion = [1;0;0;0];
else
    quaternion = quaternion/norm(quaternion);
end
if ~all(isfinite(omega)), omega = zeros(3,1); end

phase = min(max(t/cfg.takeoff_ramp_s,0),1);
frequencyCommand = cfg.takeoff_frequency_hz*(3*phase^2-2*phase^3);
if isinf(liftoffTime) && t >= cfg.takeoff_ramp_s && ...
        normalForce < cfg.liftoff_normal_threshold_N
    liftoffTime = t;
    liftoffPosition = position;
    rotation = flappy_quaternion_rotation(quaternion);
    referenceYaw = atan2(rotation(2,1),rotation(1,1));
end
isHover = ~isinf(liftoffTime);
referenceXY = liftoffPosition(1:2);
referenceZ = liftoffPosition(3);
attitudeReference = [0;0;referenceYaw];
zError = referenceZ-position(3);
if isHover
    referencePhase = min(max((t-liftoffTime)/cfg.reference_ramp_s,0),1);
    smoothPhase = 3*referencePhase^2-2*referencePhase^3;
    referenceXY = liftoffPosition(1:2) + cfg.position_reference_offset_xy_m(:)*smoothPhase;
    if t >= cfg.position_step_time_s
        referenceXY = referenceXY + cfg.position_step_delta_xy_m(:);
    end
    referenceZ = liftoffPosition(3) + (cfg.hover_height_m-liftoffPosition(3))*smoothPhase;
    if t-lastUpdate >= cfg.position_sample_time_s-1e-12
        positionKp = cfg.position_kp_s2(:);
        positionKd = cfg.position_kd_s(:);
        if cfg.disturbance.enabled
            positionKp = cfg.disturbance.position_kp_s2(:);
            positionKd = cfg.disturbance.position_kd_s(:);
        end
        accelerationXY = -positionKp.*(position(1:2)-referenceXY) - ...
            positionKd.*velocity(1:2);
        accelerationXY = min(max(accelerationXY,-cfg.horizontal_acceleration_limit_m_s2), ...
            cfg.horizontal_acceleration_limit_m_s2);
        lastUpdate = t;
    end
    horizontalScale = max(cfg.gravity_mps2, ...
        norm([accelerationXY;cfg.gravity_mps2]));
    pitchReference = asin(max(min(accelerationXY(1)/horizontalScale,0.99),-0.99));
    rollReference = -asin(max(min(accelerationXY(2)/horizontalScale/max(cos(pitchReference),1e-6),0.99),-0.99));
    attitudeReference(1:2) = min(max([rollReference;pitchReference], ...
        -cfg.attitude_reference_limit_rad(:)),cfg.attitude_reference_limit_rad(:));
    zError = referenceZ-position(3);
    [roll,pitch,~] = flappy_quaternion_to_euler(quaternion.');
    [frequencyCommand,heightDiagnostic] = flappy_sil_flappy_height_pid( ...
        referenceZ,position(3),velocity(3),roll,pitch,true,t,cfg);
else
    [~,heightDiagnostic] = flappy_sil_flappy_height_pid( ...
        referenceZ,position(3),velocity(3),0,0,false,t,cfg);
end
attitudeError = flappy_quaternion_attitude_error(quaternion, ...
    flappy_euler_to_quaternion(attitudeReference));
trimTorque = flappy_sil_attitude_trim(attitudeError,isHover,t);
torqueCommand = -cfg.attitude_kp_Nm_rad(:).*attitudeError - ...
    cfg.attitude_kd_Nms_rad(:).*omega + trimTorque;
torqueCommand = cfg.virtual_torque_axis_sign(:).*torqueCommand;
output = [frequencyCommand; torqueCommand; referenceZ; zError; attitudeError; ...
    double(isHover); accelerationXY; referenceXY; attitudeReference(1:2); trimTorque; ...
    heightDiagnostic];
end
