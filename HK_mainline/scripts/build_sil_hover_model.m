%BUILD_SIL_HOVER_MODEL Build the delayed-actuator software-in-loop hover model.

projectRoot = string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot,"src"));
cfg = flappy_sil_config(projectRoot);
run(fullfile(projectRoot,"scripts","build_ground_takeoff_model.m"));
sourceFile = fullfile(projectRoot,"models","flappy_test2_ground_takeoff.slx");
modelName = "flappy_test2_sil_hover";
modelFile = fullfile(projectRoot,"models",modelName+".slx");
if bdIsLoaded(modelName)
    close_system(modelName,0);
end
copyfile(sourceFile,modelFile,"f");
load_system(modelFile);
set_param(modelName,"Solver","ode15s","MaxStep",num2str(cfg.plant_sample_time_s), ...
    "StopTime",num2str(cfg.duration_s),"SaveOutput","on", ...
    "SaveFormat","StructureWithTime","SimMechanicsOpenEditorOnUpdate","on");

% Replace fixed workspace trajectories with online frequency-driven motion.
sourceNames = ["left_stroke_ref","left_pitch_ref","right_stroke_ref", ...
    "right_pitch_ref","left_force_ref","right_force_ref", ...
    "left_torque_ref","right_torque_ref"];
converterNames = ["ps_left_stroke","ps_left_rotate","ps_right_stroke", ...
    "ps_right_rotate","ps_left_force","ps_right_force", ...
    "ps_left_torque","ps_right_torque"];
for index = 1:numel(sourceNames)
    remove_workspace_source(modelName,sourceNames(index),converterNames(index));
end

% Sensor chain: true 6DoF state -> 500 Hz sample -> deterministic 4 ms delay.
clock = modelName+"/sil_clock";
add_block("simulink/Sources/Clock",clock,"Position",[40 700 70 720]);
sensorMux = modelName+"/sil_truth_mux";
add_block("simulink/Signal Routing/Mux",sensorMux,"Inputs","10", ...
    "Position",[390 665 415 855]);
sourceBlocks = ["pslog_body_x_m","pslog_body_y_m","pslog_body_z_m", ...
    "pslog_body_vx_mps","pslog_body_vy_mps","pslog_body_vz_mps", ...
    "pslog_body_quaternion","pslog_body_omega_rad_s"];
for index = 1:numel(sourceBlocks)
    add_line(modelName,get_outport(modelName+"/"+sourceBlocks(index)), ...
        get_inport(sensorMux,index+1),"autorouting","on");
end
ground = flappy_ground_config(flappy_parameters(projectRoot));
normalSum = modelName+"/sil_total_normal";
add_block("simulink/Math Operations/Sum",normalSum,"Inputs",repmat('+',1,numel(ground.support_names)), ...
    "Position",[260 850 285 900]);
normalBlocks = "pslog_contact_"+ground.support_names+"_normal_N";
for index = 1:numel(normalBlocks)
    add_line(modelName,get_outport(modelName+"/"+normalBlocks(index)), ...
        get_inport(normalSum,index),"autorouting","on");
end
add_line(modelName,get_outport(clock),get_inport(sensorMux,1),"autorouting","on");
add_line(modelName,get_outport(normalSum),get_inport(sensorMux,10),"autorouting","on");
sensorHold = modelName+"/sil_sensor_sample";
sensorDelay = modelName+"/sil_sensor_delay";
add_block("simulink/Discrete/Zero-Order Hold",sensorHold, ...
    "SampleTime",num2str(cfg.control_sample_time_s),"Position",[455 735 505 765]);
add_block("simulink/Discrete/Delay",sensorDelay, ...
    "DelayLength",num2str(cfg.sensor_delay_samples), ...
    "InitialCondition","0", ...
    "SampleTime",num2str(cfg.control_sample_time_s),"Position",[545 735 595 765]);
add_line(modelName,get_outport(sensorMux),get_inport(sensorHold),"autorouting","on");
add_line(modelName,get_outport(sensorHold),get_inport(sensorDelay),"autorouting","on");

% 500 Hz controller and delayed/rate-limited virtual actuator.
controller = modelName+"/sil_hover_controller";
actuatorInput = modelName+"/sil_actuator_input";
actuator = modelName+"/sil_virtual_actuator";
add_block("simulink/User-Defined Functions/Interpreted MATLAB Function",controller, ...
    "MATLABFcn","flappy_sil_controller(u)","OutputDimensions","24", ...
    "SampleTime",num2str(cfg.control_sample_time_s),"Position",[650 730 770 770]);
add_block("simulink/Signal Routing/Mux",actuatorInput,"Inputs","2", ...
    "Position",[805 725 830 775]);
add_block("simulink/User-Defined Functions/Interpreted MATLAB Function",actuator, ...
    "MATLABFcn","flappy_sil_virtual_actuator(u)","OutputDimensions","12", ...
    "SampleTime",num2str(cfg.control_sample_time_s),"Position",[865 730 990 770]);
controllerDemux = modelName+"/sil_controller_demux";
actuatorDemux = modelName+"/sil_actuator_demux";
add_block("simulink/Signal Routing/Demux",controllerDemux,"Outputs","24", ...
    "Position",[795 650 800 710]);
add_block("simulink/Signal Routing/Demux",actuatorDemux,"Outputs","12", ...
    "Position",[1020 650 1025 735]);
add_line(modelName,get_outport(sensorDelay),get_inport(controller),"autorouting","on");
add_line(modelName,get_outport(controller),get_inport(actuatorInput,2),"autorouting","on");
add_line(modelName,get_outport(clock),get_inport(actuatorInput,1),"autorouting","on");
add_line(modelName,get_outport(actuatorInput),get_inport(actuator),"autorouting","on");
add_line(modelName,get_outport(controller),get_inport(controllerDemux),"autorouting","on");
add_line(modelName,get_outport(actuator),get_inport(actuatorDemux),"autorouting","on");

% Actual actuator frequency integrates to phase and drives all wing inputs.
% The wing block also receives torso attitude, body angular velocity and world
% linear velocity.  The aero mode decides whether to use them, preserving the
% old zero-velocity baseline until a runner explicitly enables feedback.
frequencyGain = modelName+"/sil_phase_rate";
phaseIntegrator = modelName+"/sil_phase";
wingInput = modelName+"/sil_wing_input";
wingState = modelName+"/sil_wing_state";
wingStateHold = modelName+"/sil_wing_state_sample";
wingLoads = modelName+"/sil_wing_loads";
wingDemux = modelName+"/sil_wing_demux";
kinematicInput = modelName+"/sil_wing_kinematic_input";
kinematics = modelName+"/sil_wing_kinematics";
kinematicDemux = modelName+"/sil_wing_kinematic_demux";
add_block("simulink/Math Operations/Gain",frequencyGain,"Gain","2*pi", ...
    "Position",[1060 540 1110 570]);
add_block("simulink/Continuous/Integrator",phaseIntegrator,"InitialCondition","0", ...
    "Position",[1140 540 1170 570]);
add_block("simulink/Signal Routing/Mux",wingInput,"Inputs","3", ...
    "Position",[1205 515 1230 565]);
add_block("simulink/Signal Routing/Mux",kinematicInput,"Inputs","2", ...
    "Position",[1205 610 1230 645]);
% The physical joints receive angle, angular rate, and angular acceleration
% from this analytic trajectory at the 10 kHz plant rate.  Do not derive
% motion from the 1 kHz aerodynamic sample-and-hold: doing so creates
% constraint-wrench spikes at each staircase update.
add_block("simulink/User-Defined Functions/Interpreted MATLAB Function",kinematics, ...
    "MATLABFcn","flappy_sil_wing_kinematics(u)","OutputDimensions","6", ...
    "SampleTime",num2str(cfg.plant_sample_time_s), ...
    "Position",[1265 610 1375 645]);
add_block("simulink/Signal Routing/Demux",kinematicDemux,"Outputs","6", ...
    "Position",[1410 605 1415 650]);
add_block("simulink/Signal Routing/Mux",wingState,"Inputs","3", ...
    "Position",[1125 465 1150 505]);
add_block("simulink/Discrete/Delay",wingStateHold, ...
    "DelayLength","1","InitialCondition","zeros(10,1)", ...
    "SampleTime",num2str(cfg.aero_sample_time_s), ...
    "Position",[1165 465 1215 495]);
add_block("simulink/User-Defined Functions/Interpreted MATLAB Function",wingLoads, ...
    "MATLABFcn","flappy_sil_wing_loads(u)","OutputDimensions","28", ...
    "SampleTime",num2str(cfg.aero_sample_time_s), ...
    "Position",[1265 525 1375 565]);
add_block("simulink/Signal Routing/Demux",wingDemux,"Outputs","[1 1 3 3 3 3 6 8]", ...
    "Position",[1410 495 1415 585]);
add_line(modelName,get_outport(actuatorDemux,1),get_inport(frequencyGain),"autorouting","on");
add_line(modelName,get_outport(frequencyGain),get_inport(phaseIntegrator),"autorouting","on");
add_line(modelName,get_outport(phaseIntegrator),get_inport(wingInput,1),"autorouting","on");
add_line(modelName,get_outport(actuatorDemux,1),get_inport(wingInput,2),"autorouting","on");
add_line(modelName,get_outport(phaseIntegrator),get_inport(kinematicInput,1),"autorouting","on");
add_line(modelName,get_outport(actuatorDemux,1),get_inport(kinematicInput,2),"autorouting","on");
add_line(modelName,get_outport(kinematicInput),get_inport(kinematics),"autorouting","on");
add_line(modelName,get_outport(kinematics),get_inport(kinematicDemux),"autorouting","on");
add_line(modelName,get_outport(modelName+"/pslog_body_quaternion"),get_inport(wingState,1),"autorouting","on");
add_line(modelName,get_outport(modelName+"/pslog_body_omega_rad_s"),get_inport(wingState,2),"autorouting","on");
velocityMux = modelName+"/sil_world_velocity_mux";
add_block("simulink/Signal Routing/Mux",velocityMux,"Inputs","3", ...
    "Position",[1050 465 1075 505]);
add_line(modelName,get_outport(modelName+"/pslog_body_vx_mps"),get_inport(velocityMux,1),"autorouting","on");
add_line(modelName,get_outport(modelName+"/pslog_body_vy_mps"),get_inport(velocityMux,2),"autorouting","on");
add_line(modelName,get_outport(modelName+"/pslog_body_vz_mps"),get_inport(velocityMux,3),"autorouting","on");
add_line(modelName,get_outport(velocityMux),get_inport(wingState,3),"autorouting","on");
% This one-sample delay is the 1 kHz aerodynamic state sample. It intentionally
% breaks the Simscape-force -> body-state -> aerodynamic-force algebraic loop.
add_line(modelName,get_outport(wingState),get_inport(wingStateHold),"autorouting","on");
add_line(modelName,get_outport(wingStateHold),get_inport(wingInput,3),"autorouting","on");
add_line(modelName,get_outport(wingInput),get_inport(wingLoads),"autorouting","on");
add_line(modelName,get_outport(wingLoads),get_inport(wingDemux),"autorouting","on");

connect_prescribed_motion(modelName,kinematicDemux,[1 3 5],"ps_left_stroke");
connect_prescribed_motion(modelName,kinematicDemux,[2 4 6],"ps_left_rotate");
connect_prescribed_motion(modelName,kinematicDemux,[1 3 5],"ps_right_stroke");
connect_prescribed_motion(modelName,kinematicDemux,[2 4 6],"ps_right_rotate");
connect_signal_to_converter(modelName,get_outport(wingDemux,3),"ps_left_force");
connect_signal_to_converter(modelName,get_outport(wingDemux,4),"ps_right_force");
connect_signal_to_converter(modelName,get_outport(wingDemux,5),"ps_left_torque");
connect_signal_to_converter(modelName,get_outport(wingDemux,6),"ps_right_torque");

% Online aerodynamic vectors are emitted in World coordinates by
% flappy_sil_wing_loads.  This bypasses the fixed imported wing-link frame
% rotation and applies the same canonical Flappy wrench to both sides.
for side = ["left","right"]
    set_param(modelName+"/"+side+"_aero_wrench", ...
        "ForceResolutionFrame","World","TorqueResolutionFrame","World");
end

% The only virtual plant input is a body-frame torque; no extra force exists.
torqueMux = modelName+"/sil_actual_torque_mux";
torqueConverter = modelName+"/ps_sil_virtual_torque";
torqueWrench = modelName+"/sil_virtual_torque_wrench";
add_block("simulink/Signal Routing/Mux",torqueMux,"Inputs","3", ...
    "Position",[1080 760 1105 810]);
add_block("nesl_utility/Simulink-PS Converter",torqueConverter,"Unit","N*m", ...
    "Position",[1145 770 1210 800]);
add_block("sm_lib/Forces and Torques/External Force and Torque",torqueWrench, ...
    "EnableForce","off","EnableTorque","on", ...
    "TorqueResolutionFrame","AttachedFrame","Position",[1250 750 1350 815]);
for index = 1:3
    add_line(modelName,get_outport(actuatorDemux,index+1),get_inport(torqueMux,index), ...
        "autorouting","on");
end
torqueConverterPorts = get_param(torqueConverter,"PortHandles");
torqueWrenchPorts = get_param(torqueWrench,"PortHandles");
torsoPorts = get_param(modelName+"/torso","PortHandles");
add_line(modelName,get_outport(torqueMux),torqueConverterPorts.Inport,"autorouting","on");
add_line(modelName,torqueConverterPorts.RConn,torqueWrenchPorts.LConn(1),"autorouting","on");
add_line(modelName,torqueWrenchPorts.RConn,torsoPorts.LConn,"autorouting","on");

% Log the actual Simscape orientation of each frame where aerodynamic force is
% applied. Do not reconstruct it from assumed stroke/pitch rotations: imported
% URDF frame rotations and mirrored joints are part of the physical model.
worldPorts = get_param(modelName+"/World","PortHandles");
for side = ["left","right"]
    frameSensor = modelName+"/sil_"+side+"_wing_frame_sensor";
    add_block("sm_lib/Frames and Transforms/Transform Sensor",frameSensor, ...
        "SenseQ","on","MeasurementFrame","Base", ...
        "Position",[1470 150+(side=="right")*120 1560 205+(side=="right")*120]);
    sensorPorts = get_param(frameSensor,"PortHandles");
    wrenchPorts = get_param(modelName+"/"+side+"_aero_wrench","PortHandles");
    add_line(modelName,worldPorts.RConn,sensorPorts.LConn,"autorouting","on");
    add_line(modelName,wrenchPorts.RConn,sensorPorts.RConn(1),"autorouting","on");
    add_ps_signal_log(modelName,sensorPorts.RConn(2),"1", ...
        "sil_"+side+"_wing_quaternion",[1590 160+(side=="right")*120]);
end

% Persist controller, actuator, state, frequency, phase, and online wing loads.
add_signal_log(modelName,get_outport(sensorMux),"sil_truth_state",[470 900]);
add_signal_log(modelName,get_outport(sensorDelay),"sil_sensor_state",[615 900]);
add_signal_log(modelName,get_outport(controller),"sil_controller_output",[770 900]);
add_signal_log(modelName,get_outport(actuator),"sil_actuator_output",[990 900]);
add_signal_log(modelName,get_outport(phaseIntegrator),"sil_phase_rad",[1170 610]);
add_signal_log(modelName,get_outport(wingLoads),"sil_wing_output",[1375 610]);
add_signal_log(modelName,get_outport(kinematics),"sil_wing_kinematics_output",[1375 680]);

% Apply only a presentation layout.  It never changes block parameters,
% ports, or connections, and remains reproducible after every model rebuild.
flappy_apply_sil_layout(modelName);

save_system(modelName,modelFile);
close_system(modelName,0);
disp("Built "+modelFile)

function remove_workspace_source(modelName,sourceName,converterName)
source = modelName+"/"+sourceName;
converter = modelName+"/"+converterName;
if ishandle(get_param(source,"Handle"))
    sourcePorts = get_param(source,"PortHandles");
    converterPorts = get_param(converter,"PortHandles");
    delete_line(modelName,sourcePorts.Outport,converterPorts.Inport);
    delete_block(source);
end
end

function connect_signal_to_converter(modelName,signalPort,converterName)
converterPorts = get_param(modelName+"/"+converterName,"PortHandles");
add_line(modelName,signalPort,converterPorts.Inport,"autorouting","on");
end

function connect_prescribed_motion(modelName,demuxName,indices,converterName)
converter = modelName+"/"+converterName;
% Supply q, qdot, and qddot explicitly.  This is required for physically
% meaningful inertia reactions from an imposed joint motion.
set_param(converter,"FilteringAndDerivatives","provide", ...
    "UdotUserProvided","2");
converterPorts = get_param(converter,"PortHandles");
assert(numel(converterPorts.Inport)==3,"flappy:motionDerivativePorts", ...
    "Expected q/qdot/qddot inputs on %s.",converterName);
for inputIndex=1:3
    add_line(modelName,get_outport(demuxName,indices(inputIndex)), ...
        converterPorts.Inport(inputIndex),"autorouting","on");
end
end

function add_signal_log(modelName,signalPort,variableName,position)
sink = modelName+"/log_"+variableName;
add_block("simulink/Sinks/To Workspace",sink,"VariableName",variableName, ...
    "SaveFormat","Structure With Time", ...
    "Position",[position(1) position(2) position(1)+75 position(2)+25]);
sinkPorts = get_param(sink,"PortHandles");
add_line(modelName,signalPort,sinkPorts.Inport,"autorouting","on");
end

function add_ps_signal_log(modelName,physicalPort,unit,variableName,position)
converter = modelName+"/pslog_"+variableName;
sink = modelName+"/log_"+variableName;
add_block("nesl_utility/PS-Simulink Converter",converter,"Unit",unit, ...
    "Position",[position(1) position(2) position(1)+55 position(2)+25]);
add_block("simulink/Sinks/To Workspace",sink,"VariableName",variableName, ...
    "SaveFormat","Structure With Time", ...
    "Position",[position(1)+90 position(2) position(1)+165 position(2)+25]);
converterPorts = get_param(converter,"PortHandles");
sinkPorts = get_param(sink,"PortHandles");
add_line(modelName,physicalPort,converterPorts.LConn,"autorouting","on");
add_line(modelName,converterPorts.Outport,sinkPorts.Inport,"autorouting","on");
end

function port = get_outport(blockName,index)
if nargin < 2, index = 1; end
ports = get_param(blockName,"PortHandles");
port = ports.Outport(index);
end

function port = get_inport(blockName,index)
if nargin < 2, index = 1; end
ports = get_param(blockName,"PortHandles");
port = ports.Inport(index);
end
