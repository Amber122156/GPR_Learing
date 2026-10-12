function cfg = flappy_sil_active_config()
%FLAPPY_SIL_ACTIVE_CONFIG Return an optional per-run SIL configuration.
% Runners use this global only during simulation so Interpreted MATLAB
% Function blocks see the same deterministic configuration as the runner.

global FLAPPY_SIL_RUNTIME_CONFIG
if isempty(FLAPPY_SIL_RUNTIME_CONFIG)
    cfg = flappy_sil_config();
else
    cfg = FLAPPY_SIL_RUNTIME_CONFIG;
end
end
