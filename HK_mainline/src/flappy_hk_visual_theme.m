function theme = flappy_hk_visual_theme()
%FLAPPY_HK_VISUAL_THEME Presentation-only colours and material properties.
% Values are deliberately kept separate from dynamic model parameters.

theme.name = "HK paper presentation";
theme.reference_frame_mode = "key_frames";

theme.torso = localMaterial([0.12 0.14 0.17],1.00,[0.10 0.10 0.10 1],18);
theme.left_leading_edge = localMaterial([0.08 0.24 0.62],0.96,[0.18 0.22 0.32 1],22);
theme.right_leading_edge = localMaterial([0.66 0.10 0.12],0.96,[0.30 0.18 0.18 1],22);
theme.left_wing = localMaterial([0.12 0.40 0.88],0.75,[0.20 0.30 0.48 1],14);
theme.right_wing = localMaterial([0.88 0.20 0.18],0.75,[0.48 0.25 0.22 1],14);
theme.ground = localMaterial([0.93 0.94 0.96],0.98,[0.06 0.06 0.07 1],4);
theme.ground.width_m = 3.0;
theme.ground.height_m = 3.0;
end

function material = localMaterial(diffuse,opacity,specular,shininess)
material.diffuse = diffuse;
material.opacity = opacity;
material.ambient = [0.20 0.20 0.20 1];
material.specular = specular;
material.emissive = [0 0 0 1];
material.shininess = shininess;
end
