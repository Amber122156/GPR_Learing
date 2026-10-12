%BUILD_TEST2_MOTION_MODEL Build a visual Simscape replay driven by test2 motion.
%
% This explicitly prescribed-motion model keeps original Flappy URDF geometry
% and joint axes. It does not represent motor/spring/aerodynamic coupling;
% that remains isolated in flappy_dynamic_core for the next integration step.

projectRoot = string(fileparts(fileparts(mfilename('fullpath'))));
run(fullfile(projectRoot,"scripts","build_flappy_mechanics.m"));
sourceFile = fullfile(projectRoot,"models","flappy_mechanics.slx");
modelName = "flappy_test2_motion";
modelFile = fullfile(projectRoot,"models",modelName+".slx");

% Rebuilding is intentional: the preceding demo keeps this model open for
% replay, but an open SLX file cannot be overwritten by copyfile on Windows.
if bdIsLoaded(modelName)
    close_system(modelName,0);
end
copyfile(sourceFile,modelFile,"f");
load_system(modelFile);
jointNames = ["left_stroke", "left_rotate", "right_stroke", "right_rotate"];
signalNames = ["left_stroke_ref", "left_pitch_ref", "right_stroke_ref", "right_pitch_ref"];
for index = 1:numel(jointNames)
    joint = modelName+"/"+jointNames(index);
    set_param(joint,"MotionActuationMode","InputMotion", ...
        "TorqueActuationMode","ComputedTorque", ...
        "SensePosition","on","SenseVelocity","on");
    source = modelName+"/"+signalNames(index);
    converter = modelName+"/ps_"+jointNames(index);
    add_block("simulink/Sources/From Workspace",source, ...
        "VariableName",signalNames(index), ...
        "Position",[20 35+95*(index-1) 90 65+95*(index-1)]);
    % Filtering mode lets Simscape obtain smooth derivatives from the
    % prescribed position signal; 1e-6 s is negligible versus the 10 kHz
    % reference sample yet avoids declaring unavailable derivative ports.
    % A prescribed joint position needs value, velocity and acceleration, so
    % use a second-order Simscape filter to supply both derivatives.
    add_block("nesl_utility/Simulink-PS Converter",converter,"Unit","rad", ...
        "FilteringAndDerivatives","filter","SimscapeFilterOrder","2", ...
        "InputFilterTimeConstant","1e-6", ...
        "Position",[130 30+95*(index-1) 190 60+95*(index-1)]);
    sourcePorts = get_param(source,"PortHandles");
    converterPorts = get_param(converter,"PortHandles");
    jointPorts = get_param(joint,"PortHandles");
    add_line(modelName,sourcePorts.Outport,converterPorts.Inport,"autorouting","on");
    add_line(modelName,converterPorts.RConn,jointPorts.LConn(2),"autorouting","on");
end
set_param(modelName,"Solver","ode15s","MaxStep","1e-4", ...
    "StopTime","0.25","SaveOutput","on","SaveFormat","StructureWithTime");
save_system(modelName,modelFile);
close_system(modelName,0);
disp("Built "+modelFile)
