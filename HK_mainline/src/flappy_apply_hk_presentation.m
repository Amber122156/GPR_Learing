function report = flappy_apply_hk_presentation(modelName,projectRoot)
%FLAPPY_APPLY_HK_PRESENTATION Apply the non-physical HK Explorer theme.
% This function changes only geometry display properties and the graphic size
% of the existing Infinite Plane.  It creates no bodies, forces, contacts, or
% signal paths.

arguments
    modelName (1,1) string
    projectRoot (1,1) string
end

assert(bdIsLoaded(char(modelName)),"flappy:hkPresentation", ...
    "Model is not loaded: %s",modelName);
assert(isfolder(projectRoot),"flappy:hkPresentation", ...
    "Project root is not available: %s",projectRoot);

theme = flappy_hk_visual_theme();
parts = [ ...
    struct("link","torso",             "material",theme.torso,              "showFrame",true)
    struct("link","left_leading_edge", "material",theme.left_leading_edge,  "showFrame",true)
    struct("link","right_leading_edge","material",theme.right_leading_edge, "showFrame",true)
    % The imported VisualOriginTransform feeds each File Solid reference
    % frame.  Keep all five reference-frame ports exposed so that the visual
    % transform chain remains connected.  Explorer visibility is controlled
    % separately in the model tree, not by this connectivity option.
    struct("link","left_wing",         "material",theme.left_wing,          "showFrame",true)
    struct("link","right_wing",        "material",theme.right_wing,         "showFrame",true)];

for index = 1:numel(parts)
    visualBlock = modelName+"/"+parts(index).link+"/Visual";
    assert(getSimulinkBlockHandle(visualBlock) ~= -1, ...
        "flappy:hkPresentation","Missing visual block: %s",visualBlock);
    localSetMaterial(visualBlock,parts(index).material);
    set_param(visualBlock,"DoExposeReferenceFrame",localOnOff(parts(index).showFrame));
end

groundBlock = modelName+"/ground_plane";
assert(getSimulinkBlockHandle(groundBlock) ~= -1, ...
    "flappy:hkPresentation","Missing ground plane: %s",groundBlock);
set_param(groundBlock, ...
    "GraphicPlaneWidth",num2str(theme.ground.width_m), ...
    "GraphicPlaneHeight",num2str(theme.ground.height_m));
localSetMaterial(groundBlock,theme.ground);

report = struct( ...
    "model",modelName, ...
    "theme",theme.name, ...
    "reference_frame_mode",theme.reference_frame_mode, ...
    "ground_size_m",[theme.ground.width_m theme.ground.height_m], ...
    "visual_blocks",string({parts.link})', ...
    "physical_model_changed",false);
end

function localSetMaterial(block,material)
set_param(block, ...
    "GraphicVisPropType","SimpleVisualProperties", ...
    "GraphicDiffuseColor",mat2str(material.diffuse), ...
    "GraphicOpacity",num2str(material.opacity), ...
    "GraphicAmbientColor",mat2str(material.ambient), ...
    "GraphicSpecularColor",mat2str(material.specular), ...
    "GraphicEmissiveColor",mat2str(material.emissive), ...
    "GraphicShininess",num2str(material.shininess));
end

function value = localOnOff(flag)
if flag
    value = "on";
else
    value = "off";
end
end
