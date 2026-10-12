function ground = flappy_ground_config(p)
%FLAPPY_GROUND_CONFIG Fixed first-pass contact and open-loop takeoff settings.

ground.gravity_mps2 = 9.80665;
geometry = flappy_hk_ground_geometry(p);
ground.support_names = geometry.support_names;
ground.support_xy_m = geometry.support_xy_m;
ground.pad_radius_m = 0;
ground.torso_bottom_m = geometry.torso_bottom_m;
ground.static_friction = 0.6;
ground.dynamic_friction = 0.5;
ground.critical_velocity_m_s = 0.01;
ground.transition_width_m = 1e-4;
ground.mass_kg = flappy_vehicle_mass(p);
ground.static_penetration_m = 0.5e-3;
ground.normal_stiffness_N_m = ground.mass_kg*ground.gravity_mps2 / ...
    (numel(ground.support_names)*ground.static_penetration_m);
ground.normal_damping_ratio = 0.7;
ground.normal_damping_N_s_m = 2*ground.normal_damping_ratio*sqrt( ...
    (ground.mass_kg/numel(ground.support_names))*ground.normal_stiffness_N_m);
ground.initial_torso_z_m = -(ground.torso_bottom_m-ground.pad_radius_m) - ...
    ground.static_penetration_m;
ground.ramp_time_s = 0.15;
% The ground-takeoff diagnostic intentionally uses a slightly higher
% prescribed frequency than the upstream 34 Hz nominal operating point.
ground.target_frequency_hz = 38;
ground.duration_s = 0.8;
ground.sample_time_s = 1e-4;
end
