function output = flappy_sil_virtual_actuator(command)
%FLAPPY_SIL_VIRTUAL_ACTUATOR Delayed, bandwidth-limited virtual actuator.
% Input: [time; frequency_command; torque_command(3)].
% Output: [frequency_actual; torque_actual(3); delayed_frequency;
%          delayed_torque(3); torque_saturation(3); frequency_saturation].

persistent commandBuffer frequencyActual torqueActual
cfg = flappy_sil_config();
command = command(:);
t = command(1);
frequencyCommand = command(2);
torqueCommand = command(3:5).';
if isempty(commandBuffer) || t <= 1e-12
    commandBuffer = zeros(cfg.actuator_delay_samples,4);
    frequencyActual = 0;
    torqueActual = zeros(1,3);
end

frequencyClipped = min(max(frequencyCommand,cfg.frequency_limits_hz(1)), ...
    cfg.frequency_limits_hz(2));
torqueClipped = min(max(torqueCommand,-cfg.torque_limits_Nm),cfg.torque_limits_Nm);
torqueSaturation = double(abs(torqueCommand-torqueClipped) > 1e-14);
frequencySaturation = double(abs(frequencyCommand-frequencyClipped) > 1e-14);
delayed = commandBuffer(1,:);
commandBuffer = [commandBuffer(2:end,:); [frequencyClipped torqueClipped]];

frequencyAlpha = 1-exp(-cfg.control_sample_time_s/(1/(2*pi*cfg.frequency_bandwidth_hz)));
frequencyStep = frequencyAlpha*(delayed(1)-frequencyActual);
frequencyStep = min(max(frequencyStep,-cfg.frequency_rate_limit_hz_s*cfg.control_sample_time_s), ...
    cfg.frequency_rate_limit_hz_s*cfg.control_sample_time_s);
frequencyActual = min(max(frequencyActual+frequencyStep,cfg.frequency_limits_hz(1)), ...
    cfg.frequency_limits_hz(2));

torqueAlpha = 1-exp(-cfg.control_sample_time_s/(1/(2*pi*cfg.torque_bandwidth_hz)));
torqueStep = torqueAlpha*(delayed(2:4)-torqueActual);
rateStep = cfg.torque_rate_limits_Nm_s*cfg.control_sample_time_s;
torqueStep = min(max(torqueStep,-rateStep),rateStep);
torqueActual = min(max(torqueActual+torqueStep,-cfg.torque_limits_Nm),cfg.torque_limits_Nm);
output = [frequencyActual; torqueActual.'; delayed(1); delayed(2:4).'; ...
    torqueSaturation.'; frequencySaturation];
end
