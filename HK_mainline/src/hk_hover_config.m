function [cfg,info]=hk_hover_config(p,root)
arguments
 p (1,1) struct=hk_hover_defaults()
 root (1,1) string=string(fileparts(fileparts(mfilename('fullpath'))))
end
p=hk_hover_validate(p);
cfg=flappy_hk_15hz_hover_paper_config(root,p.seed,p.duration_s);
cfg.hover_height_m=p.height_m;cfg.takeoff_frequency_hz=p.frequency_hz;
cfg.takeoff_ramp_s=p.takeoff_ramp_s;cfg.reference_ramp_s=p.reference_ramp_s;
cfg.wind_world_m_s=[p.wind_x;p.wind_y;p.wind_z];
cfg.flappy_height_position_to_velocity_kp_s=p.z_pos_kp;
cfg.flappy_height_velocity_pid_kp=p.z_vel_kp;
cfg.flappy_height_velocity_pid_ki=p.z_vel_ki;
cfg.flappy_height_velocity_pid_kd_s=p.z_vel_kd;
cfg.flappy_height_velocity_pid_integral_limit_m_s=p.z_integral_limit;
cfg.flappy_height_velocity_reference_limit_m_s=p.z_velocity_limit;
cfg.flappy_height_acceleration_limit_m_s2=[-p.z_accel_limit,p.z_accel_limit];
cfg.position_kp_s2=[p.xy_kp_x;p.xy_kp_y];cfg.position_kd_s=[p.xy_kd_x;p.xy_kd_y];
cfg.horizontal_acceleration_limit_m_s2=p.xy_accel_limit;
cfg.attitude_reference_limit_rad=deg2rad([p.attitude_limit_deg;p.attitude_limit_deg]);
cfg.attitude_kp_Nm_rad=[p.roll_kp,p.pitch_kp,p.yaw_kp];
cfg.attitude_kd_Nms_rad=[p.roll_kd,p.pitch_kd,p.yaw_kd];
cfg.attitude_trim_ki_Nm_rad_s=[p.roll_ki,p.pitch_ki,p.yaw_ki];
cfg.attitude_trim_filter_time_s=p.trim_filter_s;
d=cfg.disturbance;d.enabled=p.wing_enabled||p.noise_enabled;
d.enable_wing_mismatch=p.wing_enabled;d.enable_sensor_errors=p.noise_enabled;
d.left_aero_scale=1+p.left_wing_pct/100;d.right_aero_scale=1+p.right_wing_pct/100;
d.position_noise_std_m=repmat(p.position_noise_m,3,1);
d.velocity_noise_std_mps=repmat(p.velocity_noise_mps,3,1);
d.attitude_noise_std_rad=repmat(deg2rad(p.attitude_noise_deg),3,1);
d.gyro_noise_std_rad_s=repmat(deg2rad(p.gyro_noise_deg_s),3,1);
% White noise only; no hidden random-walk bias in this GUI workflow.
for key={'position_bias_rw_m_sqrt_s','velocity_bias_rw_mps_sqrt_s', ...
 'attitude_bias_rw_rad_sqrt_s','gyro_bias_rw_rad_s_sqrt_s'}
 d.(key{1})=zeros(3,1);
end
% Existing controller uses these fields when disturbance.enabled is true.
% Keep GUI gains authoritative in BOTH branches of that controller.
d.position_kp_s2=cfg.position_kp_s2;d.position_kd_s=cfg.position_kd_s;
d.height_position_to_velocity_kp_s=p.z_pos_kp;
d.height_velocity_pid_kp=p.z_vel_kp;d.height_velocity_pid_ki=p.z_vel_ki;
cfg.disturbance=d;
dt=cfg.control_sample_time_s;
cfg.sensor_delay_samples=round(p.sensor_delay_ms/1000/dt);
cfg.sensor_delay_s=cfg.sensor_delay_samples*dt;
q=cfg.quad;q.delay_s=round(p.motor_delay_ms/1000/dt)*dt;
q.time_constant_s=p.motor_tau_ms/1000;q.voltage_V=p.voltage_V;
q.kv_rpm_V=p.motor_kv;q.deadzone=p.motor_deadzone;
q.ct_multiplier=p.ct_scale;q.cq_multiplier=p.cq_scale;q.antiwindup_gain_s=p.antiwindup;
q.actuator_mode="dynamic";q.mode="hover";q.case_name="gui_hover";
q=flappy_quad_derive(q);
q.nominal_wing_lift_N=interp1(cfg.hk_lift_map.frequency_hz,cfg.hk_lift_map.mean_lift_N,p.frequency_hz,'pchip');
cfg.quad=q;
cfg.actuator_delay_s=q.delay_s;cfg.actuator_delay_samples=round(q.delay_s/dt);
cfg.trajectory=struct('enabled',false);cfg.attitude_test=struct('enabled',false);
cfg.gui_parameters=p;
info=struct('mass_kg',cfg.mass_kg,'motor_delay_ms',1000*q.delay_s, ...
 'sensor_delay_ms',1000*cfg.sensor_delay_s,'control_period_ms',1000*dt, ...
 'nominal_wing_lift_N',q.nominal_wing_lift_N,'max_rotor_thrust_N',4*q.thrust_max_N);
end
