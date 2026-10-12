function cfg=flappy_hk_15hz_hover_paper_config(projectRoot,seed,durationS)
%FLAPPY_HK_15HZ_HOVER_PAPER_CONFIG Reproducible 15 Hz paper-hover case.
% This is deliberately a single, physically non-ideal quadrotor case.  The
% XY outer loop remains active so the shown trajectory is a controlled hover,
% not the open-loop attitude-only diagnostic used by the old comparison plot.
arguments
    projectRoot (1,1) string=string(fileparts(fileparts(mfilename('fullpath'))))
    seed (1,1) double {mustBeInteger,mustBeNonnegative}=20260908
    durationS (1,1) double {mustBePositive}=15
end

cfg=flappy_hk_15hz_actuator_comparison_config(projectRoot,"dynamic",seed,durationS);
cfg.filter_validation.freeze_xy_attitude_reference=false;
cfg.yaw_reference_mode="fixed";
cfg.yaw_reference_rad=0;
cfg.quad.actuator_mode="dynamic";
cfg.quad.case_name="paper_15Hz_hover_dynamic_seed_"+seed;

% The original free-hover gains are retained.  Their damping is sufficient
% for the 15 Hz demonstration while avoiding a needlessly aggressive tilt
% command in the trajectory figure.
cfg.position_kp_s2=[1.2;1.2];
cfg.position_kd_s=[1.6;1.6];
cfg.paper_analysis.figure_dpi=600;
end
