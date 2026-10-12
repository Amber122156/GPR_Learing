function file=build_hover_gui_model()
root=startup_mainline();name='flappy_hk_gui_hover';file=fullfile(root,'models',[name '.slx']);
assert(~bdIsLoaded(name),'hk:modelBusy','请先关闭 GUI 悬停模型再重建。');
source=fullfile(root,'models','flappy_hk_15hz_slow_fast_hover.slx');
assert(isfile(source),'hk:modelMissing','缺少悬停母模型：%s',source);
% Renaming through save_system updates model identity inside the SLX.
base='flappy_hk_15hz_slow_fast_hover';
assert(~bdIsLoaded(base),'hk:modelBusy','请先关闭悬停母模型。');
load_system(source);cleanup=onCleanup(@()close_system(base,0));
set_param(base,'SimMechanicsOpenEditorOnUpdate','off');save_system(base,file);
if bdIsLoaded(name),close_system(name,0);end
end
