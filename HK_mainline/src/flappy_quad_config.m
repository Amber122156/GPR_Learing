function cfg = flappy_quad_config(projectRoot)
% Frozen, explicitly provisional F1404/Gemfan hybrid propulsion baseline.
if nargin<1, projectRoot=string(fileparts(fileparts(mfilename('fullpath')))); end
cfg=flappy_sil_config(projectRoot);
cfg.duration_s=5; cfg.takeoff_frequency_hz=20; cfg.takeoff_ramp_s=0.5;
cfg.initial_torso_z_m=cfg.initial_torso_z_m+0.030;
cfg.reference_ramp_s=1;
q=struct();
q.arm_xy_m=0.090/sqrt(2);
q.hubs_m=[[1 1;1 -1;-1 -1;-1 1]*q.arm_xy_m,repmat(-0.0510087,4,1)];
q.reaction_sign=[-1;1;-1;1];
q.leg_extension_m=0.030;
q.voltage_V=12; q.kv_rpm_V=3800; q.no_load_voltage_V=10;
q.no_load_current_A=0.5; q.resistance_line_line_ohm=0.195;
q.resistance_equivalent_ohm=0.195;
q.time_constant_s=0.020; q.delay_s=0.010; q.deadzone=0.05;
% The physical baseline remains dynamic.  Analysis-only runs may select the
% ideal mode to remove delay and motor/propeller lag without changing the
% mixer, thrust limits, or the rigid-body plant.
q.actuator_mode="dynamic"; % dynamic | ideal
q.diameter_m=0.0508; q.pitch_m=0.0508; q.blades=3;
p=flappy_parameters(projectRoot); q.rho=p.vehicle.air_density_kg_m3;
q.ct_multiplier=1; q.cq_multiplier=1;
q.mode="hover"; % hover, static, locked, air_release
q.case_name="";
q.source_motor="https://store.tmotor.com/product/f1404-fpv-motor.html";
q.source_prop="https://www.gfprops.com/products/2023s.html";
q.source_equations="https://arxiv.org/html/1809.02472 (8),(10),(12),(16)";
q.source_dynamics="https://github.com/RflySim/CopterSim (motorT=0.02)";
q.parameter_status="UNVALIDATED: published carbon-propeller coefficients extrapolated to PC micro propeller; R_eq and response provisional";
q.antiwindup_gain_s=2;
q=flappy_quad_derive(q);
cfg.quad=q;
cfg.quad.nominal_wing_lift_N=interp1(cfg.hk_lift_map.frequency_hz,cfg.hk_lift_map.mean_lift_N,20);
% Diagnostic-only structural path.  It leaves the prescribed wings, rigid
% multibody plant, aerodynamic model, and controller untouched.  The source
% is the instantaneous wing aerodynamic wrench transmitted through the wing
% roots; the modal values are deliberately provisional until an IMU/hammer
% test identifies the real frame modes.
cfg.vibration=struct();
cfg.vibration.enabled=false;
cfg.vibration.source="wing_root_wrench_proxy_aero_plus_kinematic_inertia";
cfg.vibration.include_kinematic_inertia_proxy=true;
cfg.vibration.gain_scale=1.0;
cfg.vibration.imu_offset_body_m=[0;0;-0.010];
cfg.vibration.angular_mode_hz=[45;65;85];
cfg.vibration.angular_damping_ratio=[0.08;0.08;0.10];
cfg.vibration.angular_compliance_rad_per_Nm=[0.012;0.012;0.008];
cfg.vibration.linear_mode_hz=[45;65;85];
cfg.vibration.linear_damping_ratio=[0.10;0.10;0.12];
cfg.vibration.linear_compliance_m_per_N=[2e-4;2e-4;1e-4];
cfg.vibration.max_attitude_vibration_rad=deg2rad(12);
cfg.vibration.max_gyro_vibration_rad_s=8.0;
% Presentation only: a short steady-flight window keeps the high-frequency
% evidence readable instead of overlaying five seconds of raw data.
cfg.vibration.plot_zoom_window_s=0.25;
cfg.vibration.plot_rms_window_s=0.050;
end
