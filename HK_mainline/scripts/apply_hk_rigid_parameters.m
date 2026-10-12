%APPLY_HK_RIGID_PARAMETERS Write the single HK rigid-body source into URDF.
projectRoot = string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot,"src"));
p = flappy_parameters(projectRoot);
rigid = flappy_hk_rigid_parameters();
urdfFile = p.urdf_source;
text = fileread(urdfFile);
linkNames = fieldnames(rigid);
linkNames(strcmp(linkNames,"total_mass_kg")) = [];
for linkIndex = 1:numel(linkNames)
    linkName = linkNames{linkIndex};
    part = rigid.(linkName);
    origin = sprintf('<origin xyz = "%.12g %.12g %.12g" rpy = "0 0 0"/>',part.com_m);
    inertia = part.inertia_kg_m2;
    inertial = sprintf(['<inertial>\n            %s\n            <mass value = "%.12g"/>\n' ...
        '            <inertia ixx = "%.12g" iyy = "%.12g" izz = "%.12g" ' ...
        'ixy = "%.12g" iyz = "%.12g" ixz = "%.12g"/>\n        </inertial>'], ...
        origin,part.mass_kg,inertia(1,1),inertia(2,2),inertia(3,3), ...
        inertia(1,2),inertia(2,3),inertia(1,3));
    expression = ['(?s)(<link name\s*=\s*"' linkName '"\s*>).*?<inertial>.*?</inertial>'];
    replacement = ['$1\n        ' inertial];
    assert(~isempty(regexp(text,expression,"once")),"flappy:hkUrdfLink", ...
        "Could not locate inertial data for link %s in %s.",linkName,urdfFile);
    % Applying the same source twice is intentionally a no-op.
    text = regexprep(text,expression,replacement,"once");
end
fid = fopen(urdfFile,"w");
assert(fid >= 0,"flappy:hkUrdfWrite","Cannot write %s.",urdfFile);
cleanup = onCleanup(@() fclose(fid));
fwrite(fid,text,"char");
disp("Applied HK rigid-body data to "+urdfFile)
