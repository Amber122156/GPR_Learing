function backupFolder=install_hk_gyroY_gui(projectRoot)
% Run from this update folder after closing the old HK GUI.
% install_hk_gyroY_gui('E:\Matlab_Projections\HK_hover_GUI_20261011\HK_mainline')
source=fullfile(fileparts(mfilename('fullpath')),'HK_mainline');
assert(isfile(fullfile(projectRoot,'hk_hover_gui.m')), ...
 'hk:install','请选择包含 hk_hover_gui.m 的 HK_mainline 目录。');
backupFolder=fullfile(projectRoot,'backups',['gyroY_gui_' datestr(now,'yyyymmdd_HHMMSSFFF')]);
mkdir(fullfile(backupFolder,'src'));
files={'src/HKHoverApp.m','src/HKHoverSession.m','startup_mainline.m', ...
 'run_hover.m','mainline_source_hashes.m'};
for k=1:numel(files)
 copyfile(fullfile(projectRoot,files{k}),fullfile(backupFolder,files{k}));
end
module=fullfile(projectRoot,'HK_gyroY_integration');
if isfolder(module),copyfile(module,fullfile(backupFolder,'HK_gyroY_integration'));end
for k=1:numel(files)
 copyfile(fullfile(source,files{k}),fullfile(projectRoot,files{k}),'f');
end
copyfile(fullfile(source,'HK_gyroY_integration'),module,'f');
clear HKHoverApp HKHoverSession startup_mainline run_hover mainline_source_hashes
clear gyroY_online_config gyroY_online_init gyroY_online_step run_hk_gyroY_replay
rehash;
cd(projectRoot);startup_mainline();
fprintf('GUI update installed. Backup: %s\n',backupFolder);
fprintf('Run: app = hk_hover_gui;\n');
end
