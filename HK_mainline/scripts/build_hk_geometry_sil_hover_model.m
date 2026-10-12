%BUILD_HK_GEOMETRY_SIL_HOVER_MODEL Build the CAD-consistent HK SIL baseline.
projectRoot = string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot,"src"));
addpath(fullfile(projectRoot,"scripts"));
run(fullfile(projectRoot,"scripts","apply_hk_rigid_parameters.m"));
% Use the protected rebuild entry so the previous imported mechanism is
% archived before the HK inertial data are re-imported.
run(fullfile(projectRoot,"scripts","rebuild_from_active_hk_urdf.m"));
build_sil_variant_model("flappy_hk_geometry_sil_hover","flappy_sil_controller");
run(fullfile(projectRoot,"scripts","apply_hk_mechanics_presentation.m"));
