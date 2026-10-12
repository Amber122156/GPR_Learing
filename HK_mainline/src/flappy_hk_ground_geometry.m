function geometry = flappy_hk_ground_geometry(p)
%FLAPPY_HK_GROUND_GEOMETRY CAD-registered physical support points.
% The earlier envelope/inset approximation placed all contacts 7.5 mm out
% from the CAD lower-pad centres.  Use the extracted lowest-plane centroids
% so contact, ground clearance, and the displayed HK frame share one datum.

support = flappy_hk_ground_support_candidates(p);
geometry.support_xy_m = support.visual_registered_centres_m(:,1:2);
geometry.torso_bottom_m = support.visual_registered_bottom_m;
geometry.support_names = support.names;
geometry.frame_file = fullfile(p.project_root,"CAD","frame.STL");
geometry.pad_centre_inset_m = NaN;
geometry.extraction = support;
end
