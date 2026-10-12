%BUILD_GROUND_TAKEOFF_MODEL Add a ground plane, CAD-registered pad contact, and logs.

projectRoot = string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot,"src"));
p = flappy_parameters(projectRoot);
ground = flappy_ground_config(p);
run(fullfile(projectRoot,"scripts","build_test2_aero_model.m"));
sourceFile = fullfile(projectRoot,"models","flappy_test2_aero.slx");
modelName = "flappy_test2_ground_takeoff";
modelFile = fullfile(projectRoot,"models",modelName+".slx");

% The demo leaves its model open for Mechanics Explorer replay. Rebuilding
% deliberately replaces that generated replay model, so close it first.
if bdIsLoaded(modelName)
    close_system(modelName,0);
end
copyfile(sourceFile,modelFile,"f");
load_system(modelFile);
set_param(modelName,"Solver","ode15s","MaxStep","1e-4", ...
    "StopTime",num2str(ground.duration_s),"SaveOutput","on", ...
    "SaveFormat","StructureWithTime","SimMechanicsOpenEditorOnUpdate","on");

worldPorts = get_param(modelName+"/World","PortHandles");
torsoPorts = get_param(modelName+"/torso","PortHandles");
add_block("sm_lib/Curves and Surfaces/Infinite Plane",modelName+"/ground_plane", ...
    "GraphicPlaneWidth","0.5","GraphicPlaneHeight","0.5", ...
    "Position",[720 55 805 115]);
groundPorts = get_param(modelName+"/ground_plane","PortHandles");
add_line(modelName,worldPorts.RConn,groundPorts.LConn,"autorouting","on");

supportNames = ground.support_names;
for index = 1:numel(supportNames)
    suffix = supportNames(index);
    localPoint = [ground.support_xy_m(index,:) ...
        ground.torso_bottom_m-ground.pad_radius_m];
    transform = modelName+"/foot_"+suffix+"_frame";
    point = modelName+"/foot_"+suffix+"_point";
    contact = modelName+"/contact_"+suffix;
    y = 180+170*(index-1);
    add_block("sm_lib/Frames and Transforms/Rigid Transform",transform, ...
        "TranslationMethod","Cartesian", ...
        "TranslationCartesianOffset",mat2str(localPoint,8), ...
        "Position",[690 y 750 y+50]);
    add_block("sm_lib/Curves and Surfaces/Point",point, ...
        "Position",[790 y 835 y+45]);
    add_block("sm_lib/Forces and Torques/Spatial Contact Force",contact, ...
        "NormalStiffness",num2str(ground.normal_stiffness_N_m), ...
        "NormalDamping",num2str(ground.normal_damping_N_s_m), ...
        "NormalTransitionRegionWidth",num2str(ground.transition_width_m), ...
        "CoefficientOfStaticFriction",num2str(ground.static_friction), ...
        "CoefficientOfDynamicFriction",num2str(ground.dynamic_friction), ...
        "FrictionalCriticalVelocity",num2str(ground.critical_velocity_m_s), ...
        "SenseContactSignal","on","SensePenetrationDepth","on", ...
        "SenseNormalForceMagnitude","on","SenseFrictionalForceMagnitude","on", ...
        "ZeroCrossDetectContactStartAndEnd","on", ...
        "Position",[875 y-10 980 y+65]);
    transformPorts = get_param(transform,"PortHandles");
    pointPorts = get_param(point,"PortHandles");
    contactPorts = get_param(contact,"PortHandles");
    add_line(modelName,torsoPorts.LConn,transformPorts.LConn,"autorouting","on");
    add_line(modelName,transformPorts.RConn,pointPorts.LConn,"autorouting","on");
    add_line(modelName,groundPorts.RConn,contactPorts.LConn,"autorouting","on");
    add_line(modelName,pointPorts.RConn,contactPorts.RConn(1),"autorouting","on");
    add_ps_log(modelName,contactPorts.RConn(2),"1","contact_"+suffix+"_flag",[1020 y-5]);
    add_ps_log(modelName,contactPorts.RConn(3),"m","contact_"+suffix+"_penetration_m",[1020 y+35]);
    add_ps_log(modelName,contactPorts.RConn(4),"N","contact_"+suffix+"_normal_N",[1020 y+75]);
    add_ps_log(modelName,contactPorts.RConn(5),"N","contact_"+suffix+"_friction_N",[1020 y+115]);
end

root = modelName+"/torso_to_world";
set_param(root,"PxPositionTargetSpecify","on","PxPositionTargetValue","0", ...
    "PyPositionTargetSpecify","on","PyPositionTargetValue","0", ...
    "PzPositionTargetSpecify","on", ...
    "PzPositionTargetValue",num2str(ground.initial_torso_z_m,12), ...
    "PxSensePosition","on","PySensePosition","on","PzSensePosition","on", ...
    "PxSenseVelocity","on","PySenseVelocity","on","PzSenseVelocity","on", ...
    "SphSensePosition","on","SphSenseVelocity","on");
rootPorts = get_param(root,"PortHandles");
add_ps_log(modelName,rootPorts.RConn(2),"m","body_x_m",[1050 640]);
add_ps_log(modelName,rootPorts.RConn(3),"m/s","body_vx_mps",[1240 640]);
add_ps_log(modelName,rootPorts.RConn(4),"m","body_y_m",[1050 680]);
add_ps_log(modelName,rootPorts.RConn(5),"m/s","body_vy_mps",[1240 680]);
add_ps_log(modelName,rootPorts.RConn(6),"m","body_z_m",[1050 720]);
add_ps_log(modelName,rootPorts.RConn(7),"m/s","body_vz_mps",[1240 720]);
add_ps_log(modelName,rootPorts.RConn(8),"1","body_quaternion",[1050 760]);
add_ps_log(modelName,rootPorts.RConn(9),"rad/s","body_omega_rad_s",[1240 760]);

save_system(modelName,modelFile);
close_system(modelName,0);
disp("Built "+modelFile)

function add_ps_log(modelName,physicalPort,unit,variableName,position)
converter = modelName+"/pslog_"+variableName;
sink = modelName+"/log_"+variableName;
add_block("nesl_utility/PS-Simulink Converter",converter,"Unit",unit, ...
    "Position",[position(1) position(2) position(1)+55 position(2)+25]);
add_block("simulink/Sinks/To Workspace",sink,"VariableName",variableName, ...
    "SaveFormat","Structure With Time", ...
    "Position",[position(1)+90 position(2) position(1)+155 position(2)+25]);
converterPorts = get_param(converter,"PortHandles");
sinkPorts = get_param(sink,"PortHandles");
add_line(modelName,physicalPort,converterPorts.LConn,"autorouting","on");
add_line(modelName,converterPorts.Outport,sinkPorts.Inport,"autorouting","on");
end
