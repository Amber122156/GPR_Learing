%BUILD_TEST2_AERO_MODEL Add original whole-wing wrench inputs to the replay.
%
% The forces and moments are precomputed from the ported upstream Wing.cpp
% model using the prescribed joint state.  Each wrench is resolved in its
% attached wing frame and applied at the wing root; the moment includes the
% pressure-centre offset and rotational damping, so it is mechanically
% equivalent to applying the force at the instantaneous pressure centre.

projectRoot = string(fileparts(fileparts(mfilename('fullpath'))));
run(fullfile(projectRoot,"scripts","build_test2_motion_model.m"));
sourceFile = fullfile(projectRoot,"models","flappy_test2_motion.slx");
modelName = "flappy_test2_aero";
modelFile = fullfile(projectRoot,"models",modelName+".slx");

% This model is intentionally retained open after a replay. Close that
% disposable replay instance before regenerating its file for a downstream
% build (for example, the ground-takeoff model).
if bdIsLoaded(modelName)
    close_system(modelName,0);
end
copyfile(sourceFile,modelFile,"f");
load_system(modelFile);
for side = ["left", "right"]
    wing = modelName+"/"+side+"_wing";
    wrench = modelName+"/"+side+"_aero_wrench";
    forceSource = modelName+"/"+side+"_force_ref";
    torqueSource = modelName+"/"+side+"_torque_ref";
    forceConverter = modelName+"/ps_"+side+"_force";
    torqueConverter = modelName+"/ps_"+side+"_torque";
    add_block("sm_lib/Forces and Torques/External Force and Torque",wrench, ...
        "EnableForce","on","EnableTorque","on", ...
        "Position",[550 90+(side=="right")*240 640 150+(side=="right")*240]);
    add_block("simulink/Sources/From Workspace",forceSource, ...
        "VariableName",side+"_force_ref", ...
        "Position",[300 70+(side=="right")*240 370 100+(side=="right")*240]);
    add_block("simulink/Sources/From Workspace",torqueSource, ...
        "VariableName",side+"_torque_ref", ...
        "Position",[300 125+(side=="right")*240 370 155+(side=="right")*240]);
    add_block("nesl_utility/Simulink-PS Converter",forceConverter,"Unit","N", ...
        "Position",[410 65+(side=="right")*240 470 100+(side=="right")*240]);
    add_block("nesl_utility/Simulink-PS Converter",torqueConverter,"Unit","N*m", ...
        "Position",[410 120+(side=="right")*240 470 155+(side=="right")*240]);
    forceSourcePorts = get_param(forceSource,"PortHandles");
    torqueSourcePorts = get_param(torqueSource,"PortHandles");
    forceConverterPorts = get_param(forceConverter,"PortHandles");
    torqueConverterPorts = get_param(torqueConverter,"PortHandles");
    wrenchPorts = get_param(wrench,"PortHandles");
    wingPorts = get_param(wing,"PortHandles");
    add_line(modelName,forceSourcePorts.Outport,forceConverterPorts.Inport,"autorouting","on");
    add_line(modelName,torqueSourcePorts.Outport,torqueConverterPorts.Inport,"autorouting","on");
    add_line(modelName,forceConverterPorts.RConn,wrenchPorts.LConn(1),"autorouting","on");
    add_line(modelName,torqueConverterPorts.RConn,wrenchPorts.LConn(2),"autorouting","on");
    add_line(modelName,wrenchPorts.RConn,wingPorts.LConn,"autorouting","on");
end
set_param(modelName,"Solver","ode15s","MaxStep","1e-4", ...
    "StopTime","0.25","SaveOutput","on","SaveFormat","StructureWithTime");
save_system(modelName,modelFile);
close_system(modelName,0);
disp("Built "+modelFile)
