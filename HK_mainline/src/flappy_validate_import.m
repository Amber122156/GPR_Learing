function report = flappy_validate_import(modelName, p)
%FLAPPY_VALIDATE_IMPORT Validate imported topology and frozen upstream data.

arguments
    modelName (1,1) string
    p (1,1) struct
end

expected = ["torso_to_world","left_stroke","left_rotate","right_stroke","right_rotate"];
for name = expected
    assert(~isempty(find_system(modelName,"SearchDepth",1,"Name",name)), ...
        "flappy:import","Imported model is missing %s.",name);
end
visuals = find_system(modelName,"MaskType","File Solid");
assert(numel(visuals)==5,"flappy:import","Expected five Flappy File Solid blocks.");
assert(p.vehicle.frequency==34,"flappy:parameters","Expected upstream 34 Hz baseline.");
report.model = modelName;
report.joints = expected;
report.visual_count = numel(visuals);
report.frequency_Hz = p.vehicle.frequency;
end
