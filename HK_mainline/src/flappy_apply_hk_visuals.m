function appliedBlocks = flappy_apply_hk_visuals(modelName,projectRoot)
%FLAPPY_APPLY_HK_VISUALS Map the CAD prototype meshes onto existing links.
% This function intentionally edits only Solid visual geometry properties.
% It does not change inertial, joint, aerodynamic, or controller parameters.

arguments
    modelName (1,1) string
    projectRoot (1,1) string
end

flappy_prepare_hk_symmetric_visual_meshes(projectRoot);
meshRoot = fullfile(projectRoot,"assets","HK_prototype","meshes");

linkNames = ["left_leading_edge"; "left_wing"; "right_leading_edge"; ...
    "right_wing"; "torso"];
meshFiles = ["left_LE_symmetric_visual.STL"; ...
    "left_wing_symmetric_visual.STL"; ...
    "right_LE_symmetric_visual.STL"; ...
    "right_wing_symmetric_visual.STL"; ...
    "torso_symmetric_visual.STL"];
meshUnits = repmat("mm",5,1);

assert(bdIsLoaded(char(modelName)),"Model is not loaded: %s",modelName);
visualBlocks = find_system(char(modelName),"LookUnderMasks","all", ...
    "FollowLinks","on","Name","Visual");
appliedBlocks = strings(0,1);
for blockIndex = 1:numel(visualBlocks)
    block = string(visualBlocks{blockIndex});
    [~,parentName] = fileparts(string(get_param(block,"Parent")));
    mappingIndex = find(linkNames == parentName,1);
    if isempty(mappingIndex)
        continue
    end

    set_param(block,"ExtGeomFileName",fullfile(meshRoot,meshFiles(mappingIndex)), ...
        "ExtGeomFileUnits",meshUnits(mappingIndex));
    appliedBlocks(end+1,1) = block; %#ok<AGROW>
end

assert(numel(appliedBlocks)==numel(linkNames), ...
    "Expected %d mapped visual blocks in %s, found %d.", ...
    numel(linkNames),modelName,numel(appliedBlocks));
end
