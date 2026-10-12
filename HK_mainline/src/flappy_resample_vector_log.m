function values = flappy_resample_vector_log(simulation,variableName,time_s,width)
%FLAPPY_RESAMPLE_VECTOR_LOG Resample a vector To Workspace structure signal.

logged = simulation.get(variableName);
sourceTime = logged.time(:);
sourceValues = squeeze(logged.signals.values);
if size(sourceValues,1) == numel(sourceTime) && size(sourceValues,2) == width
    samples = sourceValues;
elseif size(sourceValues,1) == width && size(sourceValues,2) == numel(sourceTime)
    samples = sourceValues.';
else
    error("flappy:vectorLog","%s has unexpected width.",variableName);
end
values = interp1(sourceTime,samples,time_s,"linear","extrap");
end
