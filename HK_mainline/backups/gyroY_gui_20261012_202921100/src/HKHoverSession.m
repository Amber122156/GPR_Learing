classdef HKHoverSession < handle
 % Owns one model/global configuration and only its unique workspace logs.
 properties (SetAccess=private)
  Model='flappy_hk_gui_hover'
  Config
  Parameters
  Info
  Result=[]
  OutputFolder=""
  StopRequested=false
  Started=false
  Finished=false
 end
 properties (Access=private)
  PreviousConfig
  OwnsModel=false
  OwnsGlobal=false
  OriginalNames={}
  LogNames={}
 end
 methods
  function obj=HKHoverSession(p)
   root=startup_mainline();obj.Parameters=hk_hover_validate(p);
   [obj.Config,obj.Info]=hk_hover_config(obj.Parameters,root);
   loaded=find_system('SearchDepth',0,'Type','block_diagram');
   for k=1:numel(loaded)
    n=loaded{k};
    if startsWith(n,'flappy_') && ~strcmp(get_param(n,'SimulationStatus'),'stopped')
     error('hk:modelBusy','另一个 Flappy 仿真正在运行：%s',n);
    end
   end
   assert(~bdIsLoaded(obj.Model),'hk:modelBusy','GUI 模型已打开，请关闭后再运行（避免覆盖未保存更改或另一 GUI）。');
   file=fullfile(root,'models',[obj.Model '.slx']);
   if ~isfile(file),file=build_hover_gui_model();end
   try
    global FLAPPY_SIL_RUNTIME_CONFIG
    obj.PreviousConfig=FLAPPY_SIL_RUNTIME_CONFIG;obj.OwnsGlobal=true;
    FLAPPY_SIL_RUNTIME_CONFIG=obj.Config;
    clear flappy_quad_controller flappy_quad_actuator flappy_sil_wing_loads flappy_sil_sensor_model
    load_system(file);obj.OwnsModel=true;
    set_param(obj.Model,'StopTime',num2str(p.duration_s,17), ...
     'SimulationMode','normal','ReturnWorkspaceOutputs','off', ...
     'SimMechanicsOpenEditorOnUpdate','off');
    block=[obj.Model '/sil_sensor_delay'];n=obj.Config.sensor_delay_samples;
    set_param(block,'DelayLength',num2str(n),'InitialCondition',mat2str(zeros(27,max(1,n))));
    sinks=find_system(obj.Model,'LookUnderMasks','all','BlockType','ToWorkspace');
    [~,id]=fileparts(tempname);prefix=['hkgui_' matlab.lang.makeValidName(id) '_'];
    for k=1:numel(sinks)
     obj.OriginalNames{k}=get_param(sinks{k},'VariableName');
     obj.LogNames{k}=[prefix num2str(k)];
     set_param(sinks{k},'VariableName',obj.LogNames{k},'MaxDataPoints','inf','Decimation','1');
    end
   catch e,delete(obj);rethrow(e);end
  end
  function start(obj)
   assert(~obj.Started,'hk:session','本会话已经开始。');obj.Started=true;
   try,set_param(obj.Model,'SimulationCommand','start');
   catch e,delete(obj);rethrow(e);end
  end
  function [done,t]=poll(obj)
   assert(obj.Started,'hk:session','尚未开始仿真。');
   done=strcmp(get_param(obj.Model,'SimulationStatus'),'stopped');
   t=get_param(obj.Model,'SimulationTime');
  end
  function stop(obj)
   if obj.OwnsModel&&~strcmp(get_param(obj.Model,'SimulationStatus'),'stopped')
    obj.StopRequested=true;set_param(obj.Model,'SimulationCommand','stop');
   end
  end
  function result=run(obj)
   % Batch path with identical model configuration and collection.
   assert(~obj.Started,'hk:session','本会话已经开始。');obj.Started=true;
   try
    set_param(obj.Model,'ReturnWorkspaceOutputs','on');
    simulation=sim(obj.Model);result=obj.finish(simulation);
   catch e,delete(obj);rethrow(e);end
  end
  function result=finish(obj,simulation)
   assert(obj.Started&&~obj.Finished,'hk:session','无待收集的仿真。');
   assert(strcmp(get_param(obj.Model,'SimulationStatus'),'stopped'),'hk:session','请等待仿真停止。');
   data=struct();
   for k=1:numel(obj.LogNames)
    key=obj.LogNames{k};
    if nargin>=2
     if ismember(key,simulation.who),data.(obj.OriginalNames{k})=simulation.get(key);end
    elseif evalin('base',['exist(''' key ''',''var'')'])
     data.(obj.OriginalNames{k})=evalin('base',key);
    end
   end
   assert(isfield(data,'sil_truth_state'),'hk:noLogs','仿真没有产生状态日志，请查看 MATLAB 诊断信息。');
   times=data.sil_truth_state.time;
   assert(numel(times)>=2&&times(end)>0,'hk:noLogs','仿真过早结束，无法分析。');
   cfg=obj.Config;cfg.requested_duration_s=cfg.duration_s;
   % Never extrapolate a manually stopped flight to the requested end time.
   cfg.duration_s=floor((times(end)+1e-9)/cfg.plant_sample_time_s)*cfg.plant_sample_time_s;
   result=flappy_quad_collect(HKHoverLogs(data),cfg);result.simulation=[];
   if obj.StopRequested,status="stopped_by_user";
   elseif cfg.duration_s<cfg.requested_duration_s-cfg.plant_sample_time_s,status="ended_early";
   else,status="completed";end
   result.run_status=status;result.parameters=obj.Parameters;result.applied_info=obj.Info;
   result.source_hashes=mainline_source_hashes(cfg.project_root);
   result.matlab_version=version;
   folder=fullfile(cfg.project_root,'results','gui_hover',string(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));
   mkdir(folder);obj.OutputFolder=folder;result.output_folder=folder;
   save(fullfile(folder,'result.mat'),'result','-v7.3');
   writetable(result.diagnostics,fullfile(folder,'diagnostics.csv'));
   hk_hover_write_json(fullfile(folder,'parameters.json'),obj.Parameters);
   hk_hover_write_json(fullfile(folder,'applied_config.json'),cfg);
   hk_hover_write_json(fullfile(folder,'metrics.json'),struct('status',status,'metrics',result.metrics,'applied_info',obj.Info));
   obj.Result=result;obj.Finished=true;
  end
  function delete(obj)
   if obj.OwnsModel
    try
     if bdIsLoaded(obj.Model)
      if ~strcmp(get_param(obj.Model,'SimulationStatus'),'stopped'),set_param(obj.Model,'SimulationCommand','stop');end
      close_system(obj.Model,0);
     end
    catch,end
    obj.OwnsModel=false;
   end
   if obj.OwnsGlobal
    global FLAPPY_SIL_RUNTIME_CONFIG
    FLAPPY_SIL_RUNTIME_CONFIG=obj.PreviousConfig;obj.OwnsGlobal=false;
   end
   for k=1:numel(obj.LogNames)
    try,evalin('base',['clear ' obj.LogNames{k}]);catch,end
   end
  end
 end
end
