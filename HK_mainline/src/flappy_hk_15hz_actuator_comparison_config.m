function cfg=flappy_hk_15hz_actuator_comparison_config(projectRoot,actuatorMode,seed,durationS)
%FLAPPY_HK_15HZ_ACTUATOR_COMPARISON_CONFIG Fixed paper comparison baseline.
arguments
    projectRoot (1,1) string=string(fileparts(fileparts(mfilename('fullpath'))))
    actuatorMode (1,1) string {mustBeMember(actuatorMode,["ideal","dynamic"])}="dynamic"
    seed (1,1) double {mustBeInteger,mustBeNonnegative}=20260908
    durationS (1,1) double {mustBePositive}=10
end

cfg=flappy_hk_15hz_slow_fast_config(projectRoot,seed,durationS);
% Isolate the natural phase-dependent wing wrench from all deliberately
% injected uncertainties and every filter/vibration validation path.
cfg.disturbance.enabled=false;
cfg.disturbance.enable_sensor_errors=false;
cfg.disturbance.enable_wing_mismatch=false;
cfg.filter_validation.enabled=false;
% This is an attitude-only figure: position-control tilt commands would
% introduce non-periodic roll/pitch content unrelated to wing forcing.
cfg.filter_validation.freeze_xy_attitude_reference=true;
cfg.vibration.enabled=false;
cfg.yaw_reference_mode="fixed";
cfg.yaw_reference_rad=0;
cfg.quad.actuator_mode=actuatorMode;
cfg.quad.case_name="paper_15Hz_actuator_"+actuatorMode+"_seed_"+seed;
cfg.paper_analysis.figure_dpi=600;
cfg.paper_analysis.attitude_zoom_start_s=6.0;
% Exactly two fixed-frequency wing cycles: expose phase-synchronous attitude
% motion without overlaying the slower closed-loop envelope from 7.5 cycles.
cfg.paper_analysis.attitude_zoom_end_s=6.0+2/cfg.takeoff_frequency_hz;
end
