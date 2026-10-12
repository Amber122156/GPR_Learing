%BUILD_FLAPPY_MECHANICS Deterministically import and configure Flappy URDF.

projectRoot = string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot,"src"));
p = flappy_parameters(projectRoot);
modelDirectory = fullfile(projectRoot,"models");
modelName = "flappy_mechanics";
modelFile = fullfile(modelDirectory,modelName+".slx");

if ~isfile(modelFile)
    previous = pwd;
    cleanup = onCleanup(@() cd(previous));
    cd(modelDirectory);
    smimport(p.urdf_import,"ModelName",modelName);
    % smimport creates a dirty in-memory model; it does not reliably write
    % the requested SLX before this script tries to reload it.  Save the
    % active-URDF import explicitly so a clean HK rebuild is deterministic.
    save_system(modelName,modelFile);
    close_system(modelName,0);
end

load_system(modelFile);
% Apply only the selected CAD visual assets.  Dynamic properties below and
% all frozen Flappy physical/aerodynamic parameters remain unchanged.
flappy_apply_hk_visuals(modelName,projectRoot);

set_param(modelName+"/MechanismConfiguration","GravityVector","[0 0 -9.80665]");
for joint = ["left_stroke","right_stroke"]
    block = modelName+"/"+joint;
    if joint=="left_stroke", lower=p.vehicle.left_stroke_lower; upper=p.vehicle.left_stroke_upper; spring=p.vehicle.left_spring_stiffness; mid=p.vehicle.left_stroke_mid;
    else, lower=p.vehicle.right_stroke_lower; upper=p.vehicle.right_stroke_upper; spring=p.vehicle.right_spring_stiffness; mid=p.vehicle.right_stroke_mid;
    end
    set_param(block,"LowerLimitSpecify","on","LowerLimitBound",num2str(lower), ...
        "UpperLimitSpecify","on","UpperLimitBound",num2str(upper), ...
        "SpringStiffness",num2str(spring),"EquilibriumPosition",num2str(mid), ...
        "DampingCoefficient","8e-9","TorqueActuationMode","ComputedTorque");
end
for joint = ["left_rotate","right_rotate"]
    block = modelName+"/"+joint;
    if joint=="left_rotate", lower=p.vehicle.left_rotate_lower; upper=p.vehicle.left_rotate_upper;
    else, lower=p.vehicle.right_rotate_lower; upper=p.vehicle.right_rotate_upper;
    end
    set_param(block,"LowerLimitSpecify","on","LowerLimitBound",num2str(lower), ...
        "UpperLimitSpecify","on","UpperLimitBound",num2str(upper), ...
        "DampingCoefficient","1e-9","TorqueActuationMode","ComputedTorque");
end
flappy_apply_pitch_axis_offsets(modelName,p);
set_param(modelName,"Solver","ode15s","StopTime","0.02","SaveOutput","on","SaveFormat","StructureWithTime");
flappy_validate_import(modelName,p);
save_system(modelName,modelFile);
close_system(modelName,0);
disp("Built "+modelFile)
