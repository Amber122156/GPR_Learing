function build_sil_variant_model(modelName,controllerFunction)
%BUILD_SIL_VARIANT_MODEL Materialize an independent SIL model from the base.

arguments
    modelName (1,1) string
    controllerFunction (1,1) string
end
projectRoot = string(fileparts(fileparts(mfilename('fullpath'))));
requestedModelName = modelName;
requestedControllerFunction = controllerFunction;
run(fullfile(projectRoot,"scripts","build_sil_hover_model.m"));
% The base builder is a script and intentionally owns modelName/cfg in this
% workspace. Restore the function arguments before copying its output.
modelName = requestedModelName;
controllerFunction = requestedControllerFunction;
sourceFile = fullfile(projectRoot,"models","flappy_test2_sil_hover.slx");
modelFile = fullfile(projectRoot,"models",modelName+".slx");
if bdIsLoaded(modelName), close_system(modelName,0); end
copyfile(sourceFile,modelFile,"f");
load_system(modelFile);
set_param(modelName+"/sil_hover_controller","MATLABFcn",controllerFunction+"(u)");
% A SIL rebuild starts from the frozen Flappy mechanism.  Reapply the
% separately generated HK CAD visuals afterwards so rebuilding the dynamics
% never silently restores the legacy torso/wing display meshes.
hkBodyMesh = fullfile(projectRoot,"assets","HK_prototype","meshes", ...
    "torso_symmetric_visual.STL");
if isfile(hkBodyMesh)
    p = flappy_parameters(projectRoot);
    flappy_apply_pitch_axis_offsets(modelName,p);
    flappy_apply_hk_visuals(modelName,projectRoot);
end
save_system(modelName,modelFile);
close_system(modelName,0);
disp("Built "+modelFile)
end
