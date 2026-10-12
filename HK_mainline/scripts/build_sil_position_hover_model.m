%BUILD_SIL_POSITION_HOVER_MODEL Build the independent XY-position SIL model.
projectRoot = string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(projectRoot,"scripts"));
build_sil_variant_model("flappy_test2_sil_position_hover","flappy_sil_position_controller");
