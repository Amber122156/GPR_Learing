function support = flappy_hk_ground_support_candidates(p)
%FLAPPY_HK_GROUND_SUPPORT_CANDIDATES Estimate physical pads from frame.STL.
% This is diagnostic-only.  Unlike the legacy envelope/inset approximation,
% it uses vertices on the CAD lowest support plane and reports both the
% physical registration and the visual-registration alternative.

frameFile = fullfile(p.project_root,"CAD","frame.STL");
assert(isfile(frameFile),"flappy:hkFrameMesh","Missing CAD frame mesh: %s",frameFile);
mesh = stlread(frameFile);
vertices = double(mesh.Points)*1e-3;
minimum = min(vertices,[],1);
maximum = max(vertices,[],1);
spanZ = maximum(3)-minimum(3);
planeTolerance = max(0.5e-3,0.01*spanZ);
bottom = vertices(vertices(:,3) <= minimum(3)+planeTolerance,:);
assert(size(bottom,1) >= 8,"flappy:hkGroundPads", ...
    "Unable to identify sufficient vertices on the frame lower support plane.");

centreXY = 0.5*(minimum(1:2)+maximum(1:2));
% The four groups are ordered by the model body frame: +X/-X first, then
% +Y/-Y. Names are retained for compatibility with the existing contact logs.
% Match the legacy support ordering exactly: [-X,-Y], [-X,+Y],
% [+X,-Y], [+X,+Y].  The legacy names are retained only as identifiers.
signs = [-1 -1; -1 1; 1 -1; 1 1];
centresRaw = zeros(4,3);
counts = zeros(4,1);
for index = 1:4
    xMask = signs(index,1)*(bottom(:,1)-centreXY(1)) >= 0;
    yMask = signs(index,2)*(bottom(:,2)-centreXY(2)) >= 0;
    group = bottom(xMask & yMask,:);
    assert(~isempty(group),"flappy:hkGroundPads", ...
        "No lower-plane vertices found in support quadrant %d.",index);
    % Median is insensitive to triangulation density and edge tessellation.
    centresRaw(index,:) = median(group,1);
    counts(index) = size(group,1);
end

datum = [-73.6396114;-73.6396114;-81.0087]*1e-3;
support.names = ["front_left";"front_right";"rear_left";"rear_right"];
support.raw_centres_m = centresRaw;
support.bottom_vertex_count = counts;
support.lowest_plane_tolerance_m = planeTolerance;
support.physical_centres_m = centresRaw + datum.';
support.visual_registered_centres_m = support.physical_centres_m + ...
    p.vehicle.hk_frame_visual_offset_m(:).';
support.physical_bottom_m = minimum(3)+datum(3);
support.visual_registered_bottom_m = support.physical_bottom_m + ...
    p.vehicle.hk_frame_visual_offset_m(3);
end
