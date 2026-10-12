%APPLY_HK_MECHANICS_PRESENTATION Apply the repeatable HK Explorer theme.
projectRoot = string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot,"src"));
modelName = "flappy_hk_geometry_sil_hover";
modelFile = fullfile(projectRoot,"models",modelName+".slx");
assert(isfile(modelFile),"flappy:hkPresentation", ...
    "Build the HK SIL model before applying its presentation theme.");

load_system(modelFile);
cleanup = onCleanup(@() close_system(modelName,0));
report = flappy_apply_hk_presentation(modelName,projectRoot);
save_system(modelName,modelFile);
disp(report)
