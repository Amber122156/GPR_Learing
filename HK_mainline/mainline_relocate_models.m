function rows=mainline_relocate_models()
%MAINLINE_RELOCATE_MODELS Save only geometry path changes inside this package.
root=startup_mainline(); files=dir(fullfile(root,'models','*.slx'));
rows=struct('model',{},'geometry_paths',{});
for k=1:numel(files)
 file=fullfile(string(files(k).folder),string(files(k).name));
 [~,name]=fileparts(file);name=string(name);
 if bdIsLoaded(name)
  assert(strcmpi(string(get_param(name,'FileName')),file),'mainline:modelConflict', ...
   'Close the model from another project before relocating: %s',name);
  assert(strcmp(get_param(name,'Dirty'),'off'),'mainline:dirtyModel', ...
   'Save or close your edited model before relocating: %s',name);
 end
 load_system(file);blocks=find_system(name,'LookUnderMasks','all','FollowLinks','on');n=0;
 for b=string(blocks).'
  if b==name,continue;end
  params=get_param(b,'DialogParameters');
  if ~isfield(params,'ExtGeomFileName'),continue;end
  old=replace(string(get_param(b,'ExtGeomFileName')),'\','/');
  if strlength(old)==0,continue;end
  marker=strfind(lower(old),'/assets/');
  if ~isempty(marker),rel=extractAfter(old,marker(end));
  elseif startsWith(lower(old),'assets/'),rel=old;
  else,error('mainline:externalGeometry','Unmapped geometry in %s: %s',b,old);end
  assert(isfile(fullfile(root,rel)),'mainline:missingGeometry','Missing geometry: %s',rel);
  set_param(b,'ExtGeomFileName',char(rel));n=n+1;
 end
 save_system(name,file);close_system(name,0);
 rows(end+1)=struct('model',name,'geometry_paths',n); %#ok<AGROW>
end
writetable(struct2table(rows),fullfile(root,'validation','portable_models.csv'));
end

