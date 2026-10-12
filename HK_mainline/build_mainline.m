function file=build_mainline(scope)
%BUILD_MAINLINE Rebuild only the takeoff-hover mainline.
arguments
 scope (1,1) string {mustBeMember(scope,["derived","full"])}="derived"
end
root=startup_mainline();
loaded=find_system('SearchDepth',0,'Type','block_diagram');
for k=1:numel(loaded)
 assert(~startsWith(loaded{k},'flappy_'),'mainline:loadedModel','Close Flappy models before rebuilding: %s',loaded{k});
end
if scope=="full"
 run(fullfile(root,'scripts','build_hk_geometry_sil_hover_model.m'));
 run(fullfile(root,'scripts','build_hk_disturbed_position_hover_model.m'));
 build_hk_quadrotor_model();
end
build_hk_15hz_slow_fast_hover_model();mainline_relocate_models();
file=build_hover_gui_model();
end
