classdef HKHoverApp < handle
 % Programmatic native MATLAB UI (no App Designer .mlapp dependency).
 properties (SetAccess=private)
  Figure
  Result=[]
  Session=[]
  Running=false
  LastError=""
 end
 properties (Access=private)
  Editors=struct()
  Axes
  Status
  Metrics
  Notes
  Buttons=struct()
  PollTimer=[]
  Closing=false
 end
 methods
  function app=HKHoverApp(visible)
   app.Figure=uifigure('Name','HK 扑旋翼 · 起飞与悬停参数实验室', ...
    'Position',[60 60 1440 900],'Visible',visible,'Color',[.96 .97 .98]);
   app.Figure.CloseRequestFcn=@(~,~)delete(app);
   main=uigridlayout(app.Figure,[5 1]);main.RowHeight={42,40,'1x',30,28};
   main.Padding=[16 12 16 12];main.RowSpacing=10;
   title=uilabel(main,'Text','HK 扑旋翼  /  起飞与定点悬停','FontSize',23,'FontWeight','bold','FontColor',[.08 .20 .32]);
   title.Layout.Row=1;
   toolbar=uigridlayout(main,[1 6]);toolbar.Layout.Row=2;
   toolbar.Padding=[0 0 0 0];toolbar.ColumnWidth={155,155,130,155,125,'1x'};
   app.Buttons.run=uibutton(toolbar,'Text','开始仿真','BackgroundColor',[.12 .42 .66],'FontColor',[1 1 1], ...
    'ButtonPushedFcn',@(~,~)app.startSimulation());
   app.Buttons.stop=uibutton(toolbar,'Text','停止并保存已有数据','Enable','off','ButtonPushedFcn',@(~,~)app.stopSimulation());
   app.Buttons.defaults=uibutton(toolbar,'Text','恢复默认参数','ButtonPushedFcn',@(~,~)app.setParameters(hk_hover_defaults()));
   app.Buttons.save=uibutton(toolbar,'Text','保存参数 JSON','ButtonPushedFcn',@(~,~)app.saveParameters());
   app.Buttons.load=uibutton(toolbar,'Text','读取参数 JSON','ButtonPushedFcn',@(~,~)app.loadParameters());
   app.Buttons.folder=uibutton(toolbar,'Text','打开本次结果文件夹','Enable','off','ButtonPushedFcn',@(~,~)winopen(char(app.Result.output_folder)));
   body=uigridlayout(main,[1 2]);body.Layout.Row=3;body.ColumnWidth={425,'1x'};body.Padding=[0 0 0 0];
   left=uigridlayout(body,[2 1]);left.RowHeight={'1x',148};left.Padding=[0 0 0 0];
   tabs=uitabgroup(left);tabs.Layout.Row=1;
   s=hk_hover_schema();groups=unique({s.group},'stable');
   tabTitles={'工况','高度','水平','姿态','翼误差','电机','传感'};
   for g=1:numel(groups)
    tab=uitab(tabs,'Title',tabTitles{g});rows=s(strcmp({s.group},groups{g}));
    form=uigridlayout(tab,[numel(rows)+1 2]);form.ColumnWidth={'1x',95};
    form.RowHeight=[repmat({40},1,numel(rows)),{'1x'}];form.Padding=[10 12 10 8];form.RowSpacing=6;form.Scrollable='on';
    for k=1:numel(rows)
     label=uilabel(form,'Text',rows(k).label,'FontSize',12,'WordWrap','on');label.Layout.Row=k;label.Layout.Column=1;
     if islogical(rows(k).default)
      editor=uicheckbox(form,'Text','','Value',rows(k).default,'ValueChangedFcn',@(~,~)app.updateNotes());
     else
      editor=uieditfield(form,'numeric','Value',rows(k).default,'Limits',[rows(k).min rows(k).max], ...
       'ValueDisplayFormat','%.5g','ValueChangedFcn',@(~,~)app.updateNotes());
      if strcmp(rows(k).key,'seed'),editor.RoundFractionalValues='on';editor.ValueDisplayFormat='%.0f';end
     end
     editor.Layout.Row=k;editor.Layout.Column=2;editor.Tooltip=rows(k).label;
     app.Editors.(rows(k).key)=editor;
    end
   end
   app.Notes=uitextarea(left,'Editable','off','FontSize',12);app.Notes.Layout.Row=2;
   right=uigridlayout(body,[2 1]);right.RowHeight={'1x',132};right.Padding=[0 0 0 0];
   charts=uigridlayout(right,[2 2]);charts.RowHeight={'1x','1x'};charts.Padding=[0 0 0 0];
   app.Axes=gobjects(4,1);
   for k=1:4,app.Axes(k)=uiaxes(charts);app.Axes(k).FontSize=11;grid(app.Axes(k),'on');end
   app.labelAxes();
   app.Metrics=uitextarea(right,'Editable','off','FontSize',13,'Value', ...
    {'运行后显示高度/水平误差、姿态 RMS 与电机分配饱和比例。'; ...
     '统计窗口：离地 2 秒后至结束；窗口不足 1 秒时不计算 RMS。'; ...
     '参数在下一次运行生效；本次运行数据会自动保存。'});
   app.Status=uilabel(main,'Text','就绪 · 默认 15 Hz / 0.80 m / 15 s','FontSize',13);app.Status.Layout.Row=4;
   footer=uilabel(main,'Text','固定 CAD 质量 0.2032 kg · 规定刚性双翼 + 四旋翼辅助 · 螺旋桨系数尚待实测标定', ...
    'FontSize',11,'FontColor',[.35 .40 .45]);footer.Layout.Row=5;
   app.updateNotes();
  end
  function p=getParameters(app)
   p=struct();s=hk_hover_schema();
   for k=1:numel(s),p.(s(k).key)=app.Editors.(s(k).key).Value;end
   p=hk_hover_validate(p);
  end
  function setParameters(app,p)
   assert(~app.Running,'hk:running','运行期间不能修改参数。');p=hk_hover_validate(p);
   keys=fieldnames(p);for k=1:numel(keys),app.Editors.(keys{k}).Value=p.(keys{k});end
   app.updateNotes();
  end
  function startSimulation(app)
   if app.Running,return;end
   app.LastError="";
   try
    p=app.getParameters();app.Result=[];
    for k=1:4,cla(app.Axes(k));end
    app.Metrics.Value={'正在仿真；完成后显示本次指标和结果路径。'};app.setRunning(true);
    app.Status.Text='正在准备模型与气动力配置…';drawnow;
    if app.Closing,return;end
    app.Session=HKHoverSession(p);app.Session.start();
    app.PollTimer=timer('ExecutionMode','fixedSpacing','Period',.5,'BusyMode','drop', ...
     'TimerFcn',@(~,~)app.pollSimulation(),'ErrorFcn',@(~,~)app.fail(MException('hk:timer','GUI 计时器发生错误，请查看 MATLAB 诊断。')));
    start(app.PollTimer);
   catch e,app.fail(e);end
  end
  function stopSimulation(app)
   if app.Running&&~isempty(app.Session)
    app.Status.Text='正在停止；随后保存已经产生的数据…';app.Session.stop();
   end
  end
  function delete(app)
   if app.Closing,return;end
   app.Closing=true;app.clearTimer();
   if ~isempty(app.Session)&&isvalid(app.Session),delete(app.Session);end
   if ~isempty(app.Figure)&&isvalid(app.Figure),delete(app.Figure);end
  end
 end
 methods (Access=private)
  function updateNotes(app)
   p=app.getParameters();dt=2;
   wing='左右翼误差关闭';if p.wing_enabled,wing=sprintf('气动力倍率：左 %.3f / 右 %.3f',1+p.left_wing_pct/100,1+p.right_wing_pct/100);end
   app.Notes.Value={sprintf('实际电机延迟 %.0f ms / 传感延迟 %.0f ms（2 ms 取整）',round(p.motor_delay_ms/dt)*dt,round(p.sensor_delay_ms/dt)*dt); ...
    wing;'左右翼设置缩放气动力及气动力矩，不改变运动相位。'; ...
    '姿态采用 PD + 有界低通积分修正；水平位置采用 PD。'; ...
    '传感误差仅为独立白噪声；随机游走偏置关闭。'; ...
    '运行时锁定参数；曲线在仿真结束后更新。'};
  end
  function setRunning(app,value)
   app.Running=value;state='on';if value,state='off';end
   keys=fieldnames(app.Editors);for k=1:numel(keys),app.Editors.(keys{k}).Enable=state;end
   for key={'run','defaults','save','load'},app.Buttons.(key{1}).Enable=state;end
   if value,app.Buttons.stop.Enable='on';app.Buttons.folder.Enable='off';
   else,app.Buttons.stop.Enable='off';if ~isempty(app.Result),app.Buttons.folder.Enable='on';end;end
  end
  function pollSimulation(app)
   if app.Closing||~app.Running,return;end
   try
    [done,t]=app.Session.poll();
    app.Status.Text=sprintf('正在仿真 · %.3f / %.1f s（仿真时间）',t,app.Session.Config.duration_s);
    if ~done,return;end
    app.clearTimer();app.Status.Text='正在收集结果与保存文件…';drawnow;
    app.Result=app.Session.finish();delete(app.Session);app.Session=[];
    app.showResult();app.setRunning(false);
   catch e,app.fail(e);end
  end
  function showResult(app)
   d=app.Result.diagnostics;t=d.time_s;
   for k=1:4,cla(app.Axes(k));end
   plot(app.Axes(1),t,d.z_m,'LineWidth',1.3);hold(app.Axes(1),'on');
   plot(app.Axes(1),t,d.z_ref_m,'--','LineWidth',1.2);hold(app.Axes(1),'off');legend(app.Axes(1),{'高度','参考'},'Location','best');
   plot(app.Axes(2),d.x_m,d.y_m,'LineWidth',1.2);hold(app.Axes(2),'on');
   plot(app.Axes(2),d.x_ref_m(end),d.y_ref_m(end),'rx','MarkerSize',9);hold(app.Axes(2),'off');legend(app.Axes(2),{'实际位置','悬停参考'},'Location','best');
   plot(app.Axes(3),t,rad2deg([d.roll_rad,d.pitch_rad,d.yaw_rad]));legend(app.Axes(3),{'横滚','俯仰','偏航'},'Location','best');
   plot(app.Axes(4),t,[d.M1_rpm,d.M2_rpm,d.M3_rpm,d.M4_rpm]);legend(app.Axes(4),{'M1','M2','M3','M4'},'Location','best');
   app.labelAxes();m=app.Result.metrics;info=app.Result.applied_info;
   if m.valid_window
    text=sprintf('高度误差 RMS %.4f m    水平误差 RMS %.4f m    横滚/俯仰 RMS %.3f° / %.3f°', ...
     m.z_rms_m,m.xy_rms_m,m.roll_rms_deg,m.pitch_rms_deg);
    sat=sprintf('分配饱和 %.2f%%    最大连续饱和 %.3f s    最大接地穿透 %.4f m',100*m.saturation_fraction,m.max_continuous_saturation_s,m.max_penetration_m);
   else,text='统计窗口不足：离地 2 秒后还需至少 1 秒数据，RMS 尚未计算。';sat=sprintf('最大接地穿透 %.4f m',m.max_penetration_m);end
   app.Metrics.Value={text;sat;sprintf('实际电机/传感延迟 %.0f / %.0f ms    单机总质量 %.4f kg',info.motor_delay_ms,info.sensor_delay_ms,info.mass_kg); ...
    ['结果：' char(app.Result.output_folder)]};
   status='已完成';
   if m.valid_window
    if m.pass,status='已完成 · 参考阈值通过';else,status='已完成 · 参考阈值未通过';end
   end
   if app.Result.run_status=="stopped_by_user",status='已停止，部分数据已保存';
   elseif app.Result.run_status=="ended_early",status='仿真异常提前结束；已保存部分数据，请查看 MATLAB 诊断';end
   app.Status.Text=sprintf('%s · 实际时长 %.3f s · 离地 %.3f s',status,t(end),m.liftoff_s);
   for k=1:4
    try,exportgraphics(app.Axes(k),fullfile(app.Result.output_folder,sprintf('chart_%d.png',k)),'Resolution',150);
    catch e,warning('hk:plotExport','曲线保存失败：%s',e.message);end
   end
  end
  function labelAxes(app)
   titles={'高度与参考','水平位置','机身姿态','四个电机转速'};
   xs={'时间 (s)','X (m)','时间 (s)','时间 (s)'};ys={'高度 (m)','Y (m)','角度 (°)','转速 (rpm)'};
   for k=1:4,title(app.Axes(k),titles{k});xlabel(app.Axes(k),xs{k});ylabel(app.Axes(k),ys{k});grid(app.Axes(k),'on');end
  end
  function clearTimer(app)
   if ~isempty(app.PollTimer)&&isvalid(app.PollTimer)
    stop(app.PollTimer);delete(app.PollTimer);
   end
   app.PollTimer=[];
  end
  function fail(app,e)
   app.clearTimer();app.LastError=string(e.message);
   if ~isempty(app.Session)&&isvalid(app.Session),delete(app.Session);end
   app.Session=[];app.setRunning(false);app.Status.Text=['运行失败：' char(app.LastError)];
   warning('hk:gui','%s',e.message);
   if strcmp(app.Figure.Visible,'on'),uialert(app.Figure,char(app.LastError),'运行失败');end
  end
  function saveParameters(app)
   [file,path]=uiputfile('*.json','保存悬停参数','hover_parameters.json');
   if isequal(file,0),return;end
   try,hk_hover_write_json(fullfile(path,file),app.getParameters());
   catch e,uialert(app.Figure,e.message,'保存失败');end
  end
  function loadParameters(app)
   [file,path]=uigetfile('*.json','读取悬停参数');if isequal(file,0),return;end
   try,app.setParameters(jsondecode(fileread(fullfile(path,file))));
   catch e,uialert(app.Figure,e.message,'读取失败');end
  end
 end
end
