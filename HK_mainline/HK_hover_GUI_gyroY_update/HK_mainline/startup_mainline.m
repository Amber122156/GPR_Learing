function root=startup_mainline()
%STARTUP_MAINLINE Activate this self-contained package (run from any folder).
root=string(fileparts(mfilename('fullpath')));
addpath(root,fullfile(root,'src'),fullfile(root,'scripts'),fullfile(root,'tests'),fullfile(root,'models'),'-begin');
addpath(fullfile(root,'HK_gyroY_integration'),'-begin');
assert(startsWith(string(which('flappy_parameters')),root+filesep), ...
 'mainline:path','Another Flappy copy shadows this package.');
for folder=["results","validation","cache"]
 if ~isfolder(fullfile(root,folder)),mkdir(fullfile(root,folder));end
end
% Relative Solid geometry paths are resolved from this project directory.
cd(root);
Simulink.fileGenControl('set','CacheFolder',fullfile(root,'cache'), ...
 'CodeGenFolder',fullfile(root,'cache','codegen'),'createDir',true);
fprintf('HK mainline: %s\n',root);
end
