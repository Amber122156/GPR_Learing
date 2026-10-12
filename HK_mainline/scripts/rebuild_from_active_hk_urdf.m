%REBUILD_FROM_ACTIVE_HK_URDF Re-import the active editable HK URDF safely.
%
% Use after changing any <inertial> entry in assets/flappy_urdf/
% hk_flapper_import.urdf.  The existing imported mechanics model is copied
% to a dated backup before replacement; downstream prescribed-wing/SIL models
% are then regenerated from the newly imported mechanics model.

projectRoot = string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot,"src"));
addpath(fullfile(projectRoot,"scripts"));
run(fullfile(projectRoot,"scripts","apply_hk_rigid_parameters.m"));
p = flappy_parameters(projectRoot);
assert(contains(p.urdf_source,"hk_flapper_import.urdf"), ...
    "This rebuild entry is restricted to the editable HK URDF.");

modelDirectory = fullfile(projectRoot,"models");
mechanicsName = "flappy_mechanics";
mechanicsFile = fullfile(modelDirectory,mechanicsName+".slx");
backupDirectory = fullfile(modelDirectory,"backups");
if ~isfolder(backupDirectory), mkdir(backupDirectory); end

if bdIsLoaded(mechanicsName), close_system(mechanicsName,0); end
if isfile(mechanicsFile)
    stamp = string(datetime("now","Format","yyyyMMdd_HHmmss"));
    backupFile = fullfile(backupDirectory,mechanicsName+"_before_hk_"+stamp+".slx");
    copyfile(mechanicsFile,backupFile);
    delete(mechanicsFile);
    disp("Backed up "+mechanicsFile+" to "+backupFile)
end

run(fullfile(projectRoot,"scripts","build_flappy_mechanics.m"));
run(fullfile(projectRoot,"scripts","build_sil_position_hover_model.m"));
disp("Rebuilt flappy_test2_sil_position_hover from "+p.urdf_source)
