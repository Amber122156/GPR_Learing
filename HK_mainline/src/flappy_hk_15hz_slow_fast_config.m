function cfg=flappy_hk_15hz_slow_fast_config(projectRoot,seed,durationS)
%FLAPPY_HK_15HZ_SLOW_FAST_CONFIG Reproducible paper-analysis baseline.
arguments
    projectRoot (1,1) string=string(fileparts(fileparts(mfilename('fullpath'))))
    seed (1,1) double {mustBeInteger,mustBeNonnegative}=20260908
    durationS (1,1) double {mustBePositive}=10
end
cfg=flappy_quad_config(projectRoot);
cfg.duration_s=durationS;
cfg.takeoff_frequency_hz=15;
cfg.takeoff_ramp_s=0.5;
cfg.hover_height_m=0.80;
cfg.reference_ramp_s=1.0;
% Deliberately retain only the requested, already-defined uncertainties.
cfg.disturbance.enabled=true;
cfg.disturbance.enable_sensor_errors=true;
cfg.disturbance.enable_wing_mismatch=true;
cfg.disturbance.seed=seed;
cfg.filter_validation=struct('enabled',false,'freeze_xy_attitude_reference',false);
cfg.quad.nominal_wing_lift_N=interp1(cfg.hk_lift_map.frequency_hz, ...
    cfg.hk_lift_map.mean_lift_N,cfg.takeoff_frequency_hz,'pchip');
assert(isfinite(cfg.quad.nominal_wing_lift_N)&&cfg.quad.nominal_wing_lift_N>=0, ...
    'flappy:analysis15HzLift','15 Hz nominal wing lift is invalid.');
cfg.quad.case_name="paper_15Hz_seed_"+seed;
% Explicit exclusions make this baseline auditable.
cfg.vibration.enabled=false;
cfg.paper_analysis=struct( ...
    'enabled',true, ...
    'fixed_frequency_start_s',cfg.takeoff_ramp_s, ...
    'steady_statistics_start_s',8.0, ...
    'attitude_statistics_start_s',2.0, ...
    'wing_cycles_to_plot',2, ...
    'figure_dpi',600);
end
