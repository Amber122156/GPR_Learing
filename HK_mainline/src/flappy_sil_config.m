function cfg = flappy_sil_config(projectRoot)
%FLAPPY_SIL_CONFIG Fixed deterministic settings for the hover SIL baseline.

arguments
    projectRoot (1,1) string = string(fileparts(fileparts(mfilename('fullpath'))))
end

p = flappy_parameters(projectRoot);
ground = flappy_ground_config(p);
cfg.project_root = projectRoot;
cfg.mass_kg = ground.mass_kg;
cfg.gravity_mps2 = ground.gravity_mps2;
cfg.weight_N = cfg.mass_kg*cfg.gravity_mps2;
cfg.initial_torso_z_m = ground.initial_torso_z_m;
cfg.duration_s = 5;
cfg.plant_sample_time_s = 1e-4;
cfg.aero_sample_time_s = 1e-3;
cfg.control_sample_time_s = 0.002;
cfg.sensor_delay_s = 0.004;
cfg.sensor_delay_samples = 2;
cfg.actuator_delay_s = 0.010;
cfg.actuator_delay_samples = 5;
cfg.takeoff_ramp_s = 0.5;
cfg.takeoff_frequency_hz = 34;
cfg.hover_height_m = 0.80;
% Bring the height reference in over one second.  The older 0.5 s ramp and
% symmetric +/-5 m/s^2 request drove the frequency loop hard at lift-off,
% then left too much upward momentum to remove cleanly.
cfg.reference_ramp_s = 1.0;
cfg.liftoff_normal_threshold_N = 0.01*cfg.weight_N;
cfg.frequency_limits_hz = [0 40];
cfg.hover_frequency_limits_hz = [15 40];
cfg.frequency_bandwidth_hz = 10;
cfg.frequency_rate_limit_hz_s = 120;
cfg.torque_bandwidth_hz = 20;
% Fixed nonideal virtual-actuator authority: roll, pitch, yaw.  The HK
% 0.2016 kg inertial baseline requires 0.1 N*m headroom on every axis.
cfg.torque_limits_Nm = [0.2 0.2 0.1];
% Reaches the torque range in about 10 ms while retaining the 10 ms command
% delay and 20 Hz first-order bandwidth; this is not an instantaneous torque.
cfg.torque_rate_limits_Nm_s = [10.00 10.00 5.00];
% The short closed-loop sign test confirms that the native virtual-torque
% feedback sign is stabilising. Keep all physical torque axes positive.
cfg.virtual_torque_axis_sign = [1;1;1];
% 2026-09-29: halve all three P/D gains for the updated CAD inertia and
% retain the real motor delay/lag. Integral trim is deliberately unchanged.
cfg.attitude_kp_Nm_rad = [0.15 0.10 0.02];
cfg.attitude_kd_Nms_rad = [0.020 0.010 0.004];
% Low-frequency attitude trim: roll/yaw only. It rejects a constant wing
% aerodynamic bias without integrating the 38 Hz flapping disturbance.
cfg.attitude_trim_ki_Nm_rad_s = [0.1 0.1 0.02]*1e-1;
cfg.attitude_trim_limits_Nm = [0.3 0.2 0.3]*1e-1;
cfg.attitude_trim_filter_time_s = 0.05;
cfg.wind_world_m_s = [0;0;0];
cfg.enable_body_velocity_aero = true;
% Reproducible state-estimate uncertainty used only by the disturbed HK SIL
% variant. The nominal HK baseline leaves both switches off.
cfg.disturbance = struct();
cfg.disturbance.enabled = false;
cfg.disturbance.enable_sensor_errors = false;
cfg.disturbance.enable_wing_mismatch = false;
cfg.disturbance.seed = 20260908;
cfg.disturbance.monte_carlo_seeds = 1:20;
cfg.disturbance.position_noise_std_m = [0.008;0.008;0.008];
cfg.disturbance.velocity_noise_std_mps = [0.020;0.020;0.020];
cfg.disturbance.attitude_noise_std_rad = deg2rad([0.25;0.25;0.25]);
cfg.disturbance.gyro_noise_std_rad_s = [0.010;0.010;0.010];
cfg.disturbance.position_bias_rw_m_sqrt_s = [0.001;0.001;0.001];
cfg.disturbance.velocity_bias_rw_mps_sqrt_s = [0.002;0.002;0.002];
cfg.disturbance.attitude_bias_rw_rad_sqrt_s = deg2rad([0.02;0.02;0.02]);
cfg.disturbance.gyro_bias_rw_rad_s_sqrt_s = [5e-4;5e-4;5e-4];
cfg.disturbance.position_bias_limit_m = [0.020;0.020;0.020];
cfg.disturbance.velocity_bias_limit_mps = [0.050;0.050;0.050];
cfg.disturbance.attitude_bias_limit_rad = deg2rad([1.5;1.5;1.5]);
cfg.disturbance.gyro_bias_limit_rad_s = [0.020;0.020;0.020];
cfg.disturbance.left_aero_scale = 1.03;
cfg.disturbance.right_aero_scale = 0.97;
cfg.disturbance.position_kp_s2 = [5.0;5.0];
cfg.disturbance.position_kd_s = [3.4;3.4];
cfg.disturbance.height_position_to_velocity_kp_s = 2.5;
cfg.disturbance.height_velocity_pid_kp = 5.0;
cfg.disturbance.height_velocity_pid_ki = 1.0;
cfg.position_sample_time_s = 0.010;
cfg.position_kp_s2 = [1.2;1.2];
cfg.position_kd_s = [1.6;1.6];
cfg.horizontal_acceleration_limit_m_s2 = 3.0;
cfg.attitude_reference_limit_rad = deg2rad([10;10]);
cfg.position_reference_offset_xy_m = [0;0];
cfg.position_step_time_s = inf;
cfg.position_step_delta_xy_m = [0;0];
% Default hover behaviour preserves the measured heading at liftoff.  Paper
% comparisons may instead request a fixed inertial yaw reference explicitly.
cfg.yaw_reference_mode = "liftoff_heading"; % liftoff_heading | fixed
cfg.yaw_reference_rad = 0;
% Frozen from Purdue Flappy's baseline PID height structure:
% position error -> vertical-velocity target -> vertical-velocity PID ->
% common motor-voltage amplitude.  The present prescribed-wing model maps
% that voltage amplitude to frequency only at the final actuator boundary.
cfg.flappy_height_position_to_velocity_kp_s = 1.5;
cfg.flappy_height_velocity_reference_limit_m_s = 1.0;
cfg.flappy_height_velocity_pid_kp = 4.0;
cfg.flappy_height_velocity_pid_ki = 1.0;
cfg.flappy_height_velocity_pid_kd_s = 0.08;
cfg.flappy_height_velocity_pid_integral_limit_m_s = 1.0;
cfg.flappy_height_acceleration_limit_m_s2 = [-4 4];
% This prescribed-wing model has no voltage state.  The retained Flappy
% height cascade therefore produces equivalent lift directly, then inverts
% the CAD-derived periodic lift map to a flapping frequency.
cfg.hk_lift_map = flappy_hk_lift_map(projectRoot);
end
