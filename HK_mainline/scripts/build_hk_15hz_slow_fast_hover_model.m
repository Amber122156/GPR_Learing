function modelName=build_hk_15hz_slow_fast_hover_model()
%BUILD_HK_15HZ_SLOW_FAST_HOVER_MODEL Independent paper-analysis model copy.
root=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'src'));addpath(fullfile(root,'scripts'));
sourceName="flappy_hk_quadrotor_position_hover";
source=fullfile(root,'models',sourceName+'.slx');
if ~isfile(source),build_hk_quadrotor_model();end
modelName="flappy_hk_15hz_slow_fast_hover";
dest=fullfile(root,'models',modelName+'.slx');
if bdIsLoaded(modelName),close_system(modelName,0);end
copyfile(source,dest,'f');load_system(dest);
set_param(modelName,'StopTime','10','MaxStep','1e-4');
save_system(modelName,dest);close_system(modelName,0);
disp('Built '+dest);
end
