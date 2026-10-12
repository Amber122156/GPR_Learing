function trimTorque = flappy_sil_attitude_trim(attitudeError,isEnabled,time_s)
%FLAPPY_SIL_ATTITUDE_TRIM Bounded low-frequency roll/yaw integral trim.

persistent filteredError trimState
cfg = flappy_sil_active_config();
attitudeError = attitudeError(:);
if isempty(filteredError) || time_s <= 1e-12
    filteredError = zeros(3,1);
    trimState = zeros(3,1);
end
if ~isEnabled
    filteredError = zeros(3,1);
    trimState = zeros(3,1);
    trimTorque = trimState;
    return
end
alpha = 1-exp(-cfg.control_sample_time_s/cfg.attitude_trim_filter_time_s);
filteredError = filteredError + alpha*(attitudeError-filteredError);
trimState = trimState - cfg.attitude_trim_ki_Nm_rad_s(:).*filteredError* ...
    cfg.control_sample_time_s;
trimState = min(max(trimState,-cfg.attitude_trim_limits_Nm(:)), ...
    cfg.attitude_trim_limits_Nm(:));
trimTorque = trimState;
end
