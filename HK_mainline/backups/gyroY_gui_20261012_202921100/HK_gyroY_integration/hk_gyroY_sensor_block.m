function out=hk_gyroY_sensor_block(u)
% Interpreted MATLAB Function block, SampleTime = 0.002 in this HK model.
% Input: [simulation Clock; delayed sensor27]. Use Clock, not sensor(1):
% sensor(1) is delayed and repeats zero during the first few samples.
% Output: [sensor27 with only entry13 replaced; raw; osc; f; phase; valid; correction].
% First 27 outputs feed quad_controller_inputs port1; last 6 are diagnostics.
% Set global GYROY_CONFIG before sim; clear this function before a new run.
persistent s
global GYROY_CONFIG
if isempty(s) || u(1)==0, s=gyroY_online_init(); end
sensor=u(2:28);
[s,o]=gyroY_online_step(s,u(1),sensor(13),GYROY_CONFIG);
sensor(13)=o.clean;
out=[sensor;o.raw;o.osc;o.fFiltered;o.phasePred;double(o.valid);o.appliedCorrection];
end
