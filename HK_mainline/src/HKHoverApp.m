classdef HKHoverApp < handle
 % Programmatic native MATLAB UI (no App Designer .mlapp dependency).
 properties
  AlgorithmConfig
 end
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
  MotorAxes
  SignalAxes
  CompareRange
  Status
  Metrics
  Notes
  Buttons=struct()
  PollTimer=[]
  Closing=false
 end
 methods
  function app=HKHoverApp(visible)
   app.AlgorithmConfig=gyroY_online_config();
   app.Figure=uifigure('Name','HK 扑旋翼 · 起飞与悬停参数实验室', ...
    'Position',[60 60 1440 900],'Visible',visible,'Color',[.96 .97 .98]);
   app.Figure.CloseRequestFcn=@(~,~)delete(app);
   main=uigridlayout(app.Figure,[5 1]);main.RowHeight={42,40,'1x',30,28};
   main.Padding=[16 12 16 12];main.RowSpacing=10;
   title=uilabel(main,'Text','HK 扑旋翼  /  起飞与定点悬停','FontSize',23,'FontWeight','bold','FontColor',[.08 .20 .32]);
   title.Layout.Row=1;
   toolbar=uigridlayout(main,[1 7]);toolbar.Layout.Row=2;
   toolbar.Padding=[0 0 0 0];toolbar.ColumnWidth={135,115,155,130,140,125,'1x'};
   app.Buttons.run=uibutton(toolbar,'Text','开始仿真','BackgroundColor',[.12 .42 .66],'FontColor',[1 1 1], ...
    'ButtonPushedFcn',@(~,~)app.startSimulation());
   app.Buttons.algorithm=uicheckbox(toolbar,'Text','开启算法','Value',false, ...
    'Tooltip','同一机体运动回放：原始 Gyro Y 与算法处理后各运行一次控制器和电机。', ...
    'ValueChangedFcn',@(~,~)app.updateNotes());
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
   right=uigridlayout(body,[2 1]);right.RowHeight={'1x',158};right.Padding=[0 0 0 0];
   chartTabs=uitabgroup(right);chartTabs.Layout.Row=1;
   overview=uitab(chartTabs,'Title','原有输出');
   charts=uigridlayout(overview,[2 2]);charts.RowHeight={'1x','1x'};charts.Padding=[0 0 0 0];
   app.Axes=gobjects(4,1);
   for k=1:4,app.Axes(k)=uiaxes(charts);app.Axes(k).FontSize=11;grid(app.Axes(k),'on');end
   app.labelAxes();
   motors=uitab(chartTabs,'Title','逐电机对比');
   motorPanel=uigridlayout(motors,[2 1]);motorPanel.RowHeight={30,'1x'};motorPanel.Padding=[4 4 4 4];
   rangeBar=uigridlayout(motorPanel,[1 3]);rangeBar.ColumnWidth={60,140,'1x'};rangeBar.Padding=[0 0 0 0];
   uilabel(rangeBar,'Text','时间范围');
   app.CompareRange=uidropdown(rangeBar,'Items',{'全程','末尾 2 s','末尾 0.5 s'}, ...
    'Value','末尾 0.5 s','ValueChangedFcn',@(~,~)app.updateComparisonRange());
   uilabel(rangeBar,'Text','蓝虚线：接入前    橙实线：接入后（同一机体输入）','FontSize',11);
   motorGrid=uigridlayout(motorPanel,[4 3]);motorGrid.Padding=[0 0 0 0];motorGrid.RowSpacing=4;motorGrid.ColumnSpacing=4;
   app.MotorAxes=gobjects(12,1);
   for k=1:12
    app.MotorAxes(k)=uiaxes(motorGrid);app.MotorAxes(k).FontSize=10;
   end
   signals=uitab(chartTabs,'Title','算法信号');
   signalPanel=uigridlayout(signals,[2 1]);signalPanel.RowHeight={24,'1x'};signalPanel.Padding=[4 4 4 4];
   uilabel(signalPanel,'Text','时间范围随“逐电机对比”页切换；学习完成前直接使用原始 Gyro Y。','FontSize',11);
   signalGrid=uigridlayout(signalPanel,[2 2]);signalGrid.Padding=[0 0 0 0];
   app.SignalAxes=gobjects(4,1);
   for k=1:4,app.SignalAxes(k)=uiaxes(signalGrid);app.SignalAxes(k).FontSize=11;end
   app.labelComparisonAxes();
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
  function setAlgorithmEnabled(app,value)
   app.Buttons.algorithm.Value=value;app.updateNotes();
  end
  function value=getAlgorithmEnabled(app)
   value=app.Buttons.algorithm.Value;
  end
  function startSimulation(app)
   if app.Running,return;end
   app.LastError="";
   try
    p=app.getParameters();app.Result=[];
    for ax=[app.Axes;app.MotorAxes;app.SignalAxes].',cla(ax);end
    app.labelAxes();app.labelComparisonAxes();app.updateNotes();
    app.Metrics.Value={'正在仿真；完成后显示本次指标和结果路径。'};app.setRunning(true);
    app.Status.Text='正在准备模型与气动力配置…';drawnow;
    if app.Closing,return;end
    c=[];if app.getAlgorithmEnabled(),c=app.AlgorithmConfig;end
    app.Session=HKHoverSession(p,c);app.Session.start();
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
   algo='算法关闭：输出原始仿真结果。';
   if app.getAlgorithmEnabled()
    c=app.AlgorithmConfig;
    algo=sprintf('算法开启：同输入回放；H=%g / K=%g / memory=%g / tau=%g / gain=%g', ...
     c.nHarm,c.K,c.memoryCycles,c.freqTau,c.corrGain);
   end
   app.Notes.Value={sprintf('实际电机延迟 %.0f ms / 传感延迟 %.0f ms（2 ms 取整）',round(p.motor_delay_ms/dt)*dt,round(p.sensor_delay_ms/dt)*dt); ...
    wing;'左右翼设置缩放气动力及气动力矩，不改变运动相位。'; ...
    '姿态采用 PD + 有界低通积分修正；水平位置采用 PD。'; ...
    '传感误差仅为独立白噪声；随机游走偏置关闭。'; ...
    algo;'运行时锁定参数；曲线在仿真与回放结束后更新。'};
  end
  function setRunning(app,value)
   app.Running=value;state='on';if value,state='off';end
   keys=fieldnames(app.Editors);for k=1:numel(keys),app.Editors.(keys{k}).Enable=state;end
   for key={'run','algorithm','defaults','save','load'},app.Buttons.(key{1}).Enable=state;end
   if value,app.Buttons.stop.Enable='on';app.Buttons.folder.Enable='off';
   else,app.Buttons.stop.Enable='off';if ~isempty(app.Result),app.Buttons.folder.Enable='on';end;end
  end
  function pollSimulation(app)
   if app.Closing||~app.Running,return;end
   try
    [done,t]=app.Session.poll();
    app.Status.Text=sprintf('正在仿真 · %.3f / %.1f s（仿真时间）',t,app.Session.Config.duration_s);
    if ~done,return;end
    app.clearTimer();app.Buttons.stop.Enable='off';
    app.Status.Text='正在收集结果与保存文件…';
    if ~isempty(app.Session.AlgorithmConfig)
     app.Status.Text='正在保存基准仿真并计算算法对比；回放期间请等待…';
    end
    drawnow limitrate nocallbacks;
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
   hasAlgorithm=isfield(app.Result,'gyroYReplay');
   if hasAlgorithm
    R=app.Result.gyroYReplay;colors=lines(4);hold(app.Axes(4),'on');
    for k=1:4
     plot(app.Axes(4),R.t,R.baseline.actuator(:,12+k)*60/(2*pi),'--','Color',colors(k,:), ...
      'DisplayName',sprintf('M%d 前',k));
     plot(app.Axes(4),R.t,R.algorithm.actuator(:,12+k)*60/(2*pi),'-','Color',colors(k,:), ...
      'DisplayName',sprintf('M%d 后',k));
    end
    hold(app.Axes(4),'off');legend(app.Axes(4),'show','Location','best','NumColumns',2);
   else
    plot(app.Axes(4),t,[d.M1_rpm,d.M2_rpm,d.M3_rpm,d.M4_rpm]);legend(app.Axes(4),{'M1','M2','M3','M4'},'Location','best');
   end
   app.labelAxes();m=app.Result.metrics;info=app.Result.applied_info;
   if m.valid_window
    text=sprintf('高度误差 RMS %.4f m    水平误差 RMS %.4f m    横滚/俯仰 RMS %.3f° / %.3f°', ...
     m.z_rms_m,m.xy_rms_m,m.roll_rms_deg,m.pitch_rms_deg);
    sat=sprintf('分配饱和 %.2f%%    最大连续饱和 %.3f s    最大接地穿透 %.4f m',100*m.saturation_fraction,m.max_continuous_saturation_s,m.max_penetration_m);
   else,text='统计窗口不足：离地 2 秒后还需至少 1 秒数据，RMS 尚未计算。';sat=sprintf('最大接地穿透 %.4f m',m.max_penetration_m);end
   app.Metrics.Value={text;sat;sprintf('实际电机/传感延迟 %.0f / %.0f ms    单机总质量 %.4f kg',info.motor_delay_ms,info.sensor_delay_ms,info.mass_kg); ...
    ['结果：' char(app.Result.output_folder)]};
   if hasAlgorithm
    app.showAlgorithmResult();
    title(app.Axes(1),'高度与参考（基准轨迹）');title(app.Axes(2),'水平位置（基准轨迹）');
    title(app.Axes(3),'机身姿态（基准轨迹）');title(app.Axes(4),'四个电机转速（同输入对比）');
    if R.analysisStatus=="ready"
     rows=R.metrics.quantity=="rpm";
     line=sprintf('转速谐波带 RMS 降幅 M1–M4：%.2f%% / %.2f%% / %.2f%% / %.2f%%（%.2f–%.2f s）', ...
      R.metrics.bandReduction_pct(rows),R.analysisWindow);
    elseif R.analysisStatus=="no_prediction"
     line='算法尚未形成有效预测：当前接入后曲线为原始旁路，未计算改善指标。';
    else
     line='算法已有输出；稳态统计窗口不足 1 s，未计算改善指标。';
    end
    app.Metrics.Value=[app.Metrics.Value;{line;'算法为相同机体输入回放；上方高度、位置与姿态指标属于基准飞行。'}];
   end
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
   if hasAlgorithm
    for k=1:12
     exportgraphics(app.MotorAxes(k),fullfile(R.outputFolder,sprintf('motor_compare_%02d.png',k)),'Resolution',150);
    end
    for k=1:4
     exportgraphics(app.SignalAxes(k),fullfile(R.outputFolder,sprintf('algorithm_signal_%d.png',k)),'Resolution',150);
    end
   end
  end
  function showAlgorithmResult(app)
   R=app.Result.gyroYReplay;t=R.t;
   columns=[16 12 24];scale=[1 60/(2*pi) 1];
   for motor=1:4
    for j=1:3
     ax=app.MotorAxes(3*(motor-1)+j);
     app.plotPair(ax,t,R.baseline.actuator(:,columns(j)+motor)*scale(j), ...
      R.algorithm.actuator(:,columns(j)+motor)*scale(j));
    end
   end
   app.plotPair(app.SignalAxes(1),t,R.sensorInput(:,13),R.algorithm.feedbackY);
   plot(app.SignalAxes(2),t,R.algorithm.log(:,4),'Color',[.1 .55 .35]);
   app.plotPair(app.SignalAxes(3),t,R.baseline.controller(:,3),R.algorithm.controller(:,3));
   app.plotPair(app.SignalAxes(4),t,R.baseline.actuator(:,3),R.algorithm.actuator(:,3));
   if isfinite(R.firstPrediction_s)
    xline(app.SignalAxes(1),R.firstPrediction_s,':','首次有效预测','DisplayName','首次有效预测');
   end
   app.labelComparisonAxes();app.updateComparisonRange();
  end
  function plotPair(~,ax,t,before,after)
   plot(ax,t,before,'--','Color',[0 .447 .741],'LineWidth',1);hold(ax,'on');
   plot(ax,t,after,'-','Color',[.85 .325 .098],'LineWidth',1.1);hold(ax,'off');
   legend(ax,{'接入前','接入后'},'Location','best','FontSize',9);
  end
  function updateComparisonRange(app)
   if isempty(app.Result)||~isfield(app.Result,'gyroYReplay'),return;end
   t=app.Result.gyroYReplay.t;
   switch app.CompareRange.Value
    case '全程',window=[t(1),t(end)];
    case '末尾 2 s',window=[max(t(1),t(end)-2),t(end)];
    case '末尾 0.5 s',window=[max(t(1),t(end)-.5),t(end)];
   end
   for ax=[app.MotorAxes;app.SignalAxes].',xlim(ax,window);end
  end
  function labelComparisonAxes(app)
   quantities={'油门','转速','推力'};units={'油门 (0–1)','转速 (rpm)','推力 (N)'};
   for motor=1:4
    for j=1:3
     ax=app.MotorAxes(3*(motor-1)+j);title(ax,sprintf('M%d %s',motor,quantities{j}));
     xlabel(ax,'时间 (s)');ylabel(ax,units{j});grid(ax,'on');
    end
   end
   titles={'Gyro Y：原始与分离后','估计的周期分量','俯仰力矩指令','四旋翼实际俯仰力矩'};
   units={'角速度 (rad/s)','角速度 (rad/s)','力矩 (N·m)','力矩 (N·m)'};
   for k=1:4
    title(app.SignalAxes(k),titles{k});xlabel(app.SignalAxes(k),'时间 (s)');
    ylabel(app.SignalAxes(k),units{k});grid(app.SignalAxes(k),'on');
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
