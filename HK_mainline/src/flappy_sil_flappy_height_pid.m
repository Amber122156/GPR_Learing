function [frequencyCommand,diagnostic] = flappy_sil_flappy_height_pid( ...
    referenceZ,positionZ,velocityZ,roll,pitch,isActive,time,cfg)
%FLAPPY_SIL_FLAPPY_HEIGHT_PID Port of the Flappy baseline PID Z-control.
% The upstream structure is retained through its common-amplitude output.
% Here that output is only an equivalent lift command: no electrical or
% motor subsystem is modeled before conversion to prescribed-wing frequency.

persistent integralVelocityError previousVelocityError
if isempty(integralVelocityError) || time <= 1e-12 || ~isActive
    integralVelocityError = 0;
    previousVelocityError = 0;
end

positionKp = cfg.flappy_height_position_to_velocity_kp_s;
velocityKp = cfg.flappy_height_velocity_pid_kp;
velocityKi = cfg.flappy_height_velocity_pid_ki;
if cfg.disturbance.enabled
    positionKp = cfg.disturbance.height_position_to_velocity_kp_s;
    velocityKp = cfg.disturbance.height_velocity_pid_kp;
    velocityKi = cfg.disturbance.height_velocity_pid_ki;
end
positionError = referenceZ-positionZ;
velocityReference = min(max(positionKp*positionError, ...
    -cfg.flappy_height_velocity_reference_limit_m_s), ...
    cfg.flappy_height_velocity_reference_limit_m_s);
velocityError = velocityReference-velocityZ;
integralVelocityError = min(max(integralVelocityError + ...
    velocityError*cfg.control_sample_time_s, ...
    -cfg.flappy_height_velocity_pid_integral_limit_m_s), ...
    cfg.flappy_height_velocity_pid_integral_limit_m_s);
velocityErrorRate = (velocityError-previousVelocityError)/cfg.control_sample_time_s;
previousVelocityError = velocityError;

accelerationCommand = velocityKp*velocityError + ...
    velocityKi*integralVelocityError + ...
    cfg.flappy_height_velocity_pid_kd_s*velocityErrorRate;
accelerationCommand = min(max(accelerationCommand, ...
    cfg.flappy_height_acceleration_limit_m_s2(1)), ...
    cfg.flappy_height_acceleration_limit_m_s2(2));
tiltScale = max(cos(roll)*cos(pitch),1e-3);
equivalentForce = cfg.mass_kg*(cfg.gravity_mps2+accelerationCommand)/tiltScale;
equivalentForce = min(max(equivalentForce,cfg.hk_lift_map.minimum_force_N), ...
    cfg.hk_lift_map.maximum_force_N);
frequencyCommand = interp1(cfg.hk_lift_map.mean_lift_N, ...
    cfg.hk_lift_map.frequency_hz,equivalentForce,"pchip");
frequencyCommand = min(max(frequencyCommand,cfg.hover_frequency_limits_hz(1)), ...
    cfg.hover_frequency_limits_hz(2));
diagnostic = [equivalentForce;velocityReference;velocityError; ...
    integralVelocityError;equivalentForce];
end
