%BUILD_HK_DISTURBED_POSITION_HOVER_MODEL Materialize the independent HK disturbance SIL.
projectRoot = string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot,"src")); addpath(fullfile(projectRoot,"scripts"));
sourceName = "flappy_hk_geometry_sil_hover";
sourceFile = fullfile(projectRoot,"models",sourceName+".slx");
if ~isfile(sourceFile)
    run(fullfile(projectRoot,"scripts","build_hk_geometry_sil_hover_model.m"));
end
modelName = "flappy_hk_disturbed_position_hover";
modelFile = fullfile(projectRoot,"models",modelName+".slx");
if bdIsLoaded(modelName), close_system(modelName,0); end
copyfile(sourceFile,modelFile,"f"); load_system(modelFile);
set_param(modelName+"/sil_hover_controller","MATLABFcn","flappy_sil_position_controller(u)");

% Truth -> 500 Hz sample -> navigation error model -> fixed 4 ms delay -> controller.
sample = modelName+"/sil_sensor_sample";
delay = modelName+"/sil_sensor_delay";
delayLines = get_param(delay,"LineHandles");
if delayLines.Inport ~= -1, delete_line(delayLines.Inport); end
sensorModel = modelName+"/sil_navigation_sensor";
add_block("simulink/User-Defined Functions/Interpreted MATLAB Function",sensorModel, ...
    "MATLABFcn","flappy_sil_sensor_model(u)","OutputDimensions","27", ...
    "SampleTime",num2str(flappy_sil_config(projectRoot).control_sample_time_s), ...
    "Position",[525 690 640 725]);
set_param(delay,"InitialCondition","zeros(27,2)");
samplePorts = get_param(sample,"PortHandles");
sensorPorts = get_param(sensorModel,"PortHandles");
delayPorts = get_param(delay,"PortHandles");
add_line(modelName,samplePorts.Outport,sensorPorts.Inport,"autorouting","on");
add_line(modelName,sensorPorts.Outport,delayPorts.Inport,"autorouting","on");
save_system(modelName,modelFile); close_system(modelName,0);
disp("Built "+modelFile)
