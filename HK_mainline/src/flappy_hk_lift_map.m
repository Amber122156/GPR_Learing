function liftMap = flappy_hk_lift_map(projectRoot)
%FLAPPY_HK_LIFT_MAP Period-average static-air lift map for prescribed wings.

arguments
    projectRoot (1,1) string = string(fileparts(fileparts(mfilename('fullpath'))))
end

p = flappy_parameters(projectRoot);
persistent cachedRoot cachedSignature cachedMap
meshInfo = dir(p.hk.aero.mesh_file);
meshSignature = string(meshInfo.datenum)+"_"+string(meshInfo.bytes);
if ~isempty(cachedMap) && cachedRoot == projectRoot && ...
        cachedSignature == meshSignature
    liftMap = cachedMap;
    return
end
% Cover the physical HK operating range.  The quadrotor mixer needs this
% map at 15 Hz as well as at the earlier 20 Hz baseline.
frequency = (10:0.25:42).';
phase = linspace(0,2*pi,721).';
meanLift = zeros(size(frequency));
for frequencyIndex = 1:numel(frequency)
    f = frequency(frequencyIndex);
    fz = zeros(size(phase));
    for phaseIndex = 1:numel(phase)
        q = phase(phaseIndex);
        stroke = deg2rad(70)*sin(q);
        strokeRate = deg2rad(70)*cos(q)*2*pi*f;
        z = 4*cos(q);
        pitch = -deg2rad(45)*tanh(z);
        pitchRate = deg2rad(45)*4*sin(q)/(cosh(z)^2)*2*pi*f;
        left = flappy_wing_aero(zeros(6,1),stroke,strokeRate,pitch,pitchRate,"left",p);
        right = flappy_wing_aero(zeros(6,1),stroke,strokeRate,pitch,pitchRate,"right",p);
        wrench = flappy_nominal_world_wrench(left,right,stroke,stroke,pitch,pitch,p);
        fz(phaseIndex) = wrench.force_world_N(3);
    end
    meanLift(frequencyIndex) = trapz(phase,fz)/(2*pi);
end
assert(all(isfinite(meanLift)) && all(diff(meanLift) > 0), ...
    "flappy:hkLiftMap", "HK static-air lift map must be finite and strictly increasing.");
liftMap = struct("frequency_hz",frequency,"mean_lift_N",meanLift, ...
    "minimum_force_N",meanLift(1),"maximum_force_N",meanLift(end));
cachedRoot = projectRoot;
cachedSignature = meshSignature;
cachedMap = liftMap;
end
