function OUT=run_hk_gyroY_replay(baselineInput,userCfg)
% Fixed-input controller/motor replay, using the unchanged HK functions.
% Place this folder directly under HK_mainline. No Simscape solve is needed.
% OUT=run_hk_gyroY_replay('.../20261012_baseline/result.mat',c);
% baselineInput may also be the collected result struct used by the GUI.
% c.outputDir selects the result folder; an empty value creates a new folder.
% Existing sensor log ALREADY includes the sensor delay: do not delay twice.
% Wing feedback and actuator feedback each retain their original one-step delay.
% This is a replay comparison, not a second freely evolving flight trajectory.
if nargin<2, userCfg=struct; end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'src'));
if isstruct(baselineInput)
    b=baselineInput; baselineFile='';
else
    baselineFile=baselineInput;
    loaded=load(baselineFile,'result'); b=loaded.result;
end
cfg=b.config; cfg.project_root=string(root);
c=gyroY_online_config(userCfg);
dt=cfg.control_sample_time_s;
stride=round(dt/cfg.plant_sample_time_s);
ix=(1:stride:height(b.diagnostics)).';
t=b.diagnostics.time_s(ix);
sensor=b.sensor(ix,:); wing=b.wing(ix,:);
wingDelayed=[zeros(1,28);wing(1:end-1,:)];
referenceController=b.controller(ix,:);
referenceActuator=b.actuator(ix,:);
fprintf('Fixed-input replay: %.0f Hz, %d samples, %.3f s\n',1/dt,numel(t),t(end));
fprintf('H=%d K=%d memory=%g tau=%g gain=%g; FFT window=%.3f s\n', ...
    c.nHarm,c.K,c.memoryCycles,c.freqTau,c.corrGain,c.nFFT*dt);

global FLAPPY_SIL_RUNTIME_CONFIG
previousCfg=FLAPPY_SIL_RUNTIME_CONFIG;
cleanup=onCleanup(@()restoreConfig(previousCfg));
FLAPPY_SIL_RUNTIME_CONFIG=cfg;
baseline=runChannel(false,t,sensor,wingDelayed,c);
% This gate verifies the experiment, rather than disguising a replay mismatch.
controllerError=max(abs(baseline.controller-referenceController),[],'all');
actuatorError=max(abs(baseline.actuator-referenceActuator),[],'all');
relativeControllerError=max(abs(baseline.controller-referenceController)./(1+abs(referenceController)),[],'all');
relativeActuatorError=max(abs(baseline.actuator-referenceActuator)./(1+abs(referenceActuator)),[],'all');
check=table(controllerError,actuatorError,relativeControllerError,relativeActuatorError);
disp(check);
assert(relativeControllerError<1e-7 && relativeActuatorError<1e-7, ...
    'Baseline replay does not match the saved run. Resolve timing/source differences before comparison.');

algorithm=runChannel(true,t,sensor,wingDelayed,c);
% Use a common steady window after learning; include all samples in saved data.
firstValid=find(algorithm.log(:,6)>0.5,1);
if isempty(firstValid)
    firstPrediction_s=NaN; window=[NaN NaN]; analysisStatus="no_prediction";
else
    firstPrediction_s=t(firstValid);
    window=[max(8,firstPrediction_s+1),t(end)];
    analysisStatus="ready";
    if window(2)-window(1)<1
        analysisStatus="insufficient_window";
    end
end
metrics=scoreMotors(t,baseline.actuator,algorithm.actuator,window,cfg.takeoff_frequency_hz,c);
OUT=struct('t',t,'config',cfg,'algorithmConfig',c,'sensorInput',sensor, ...
    'wingInput',wingDelayed,'baseline',baseline,'algorithm',algorithm, ...
    'baselineCheck',check,'analysisWindow',window,'metrics',metrics, ...
    'sourceBaseline',baselineFile,'firstPrediction_s',firstPrediction_s, ...
    'analysisStatus',analysisStatus);
folder=c.outputDir;
if isempty(folder)
    folder=fullfile(root,'results',['gyroY_replay_' datestr(now,'yyyymmdd_HHMMSSFFF')]);
end
mkdir(folder); OUT.outputFolder=folder;
save(fullfile(folder,'replay.mat'),'OUT','-v7.3');
writetable(check,fullfile(folder,'baseline_check.csv'));
writetable(metrics,fullfile(folder,'motor_metrics.csv'));
writeChannel(baseline,t,fullfile(folder,'baseline_motor.csv'));
writeChannel(algorithm,t,fullfile(folder,'algorithm_motor.csv'));
names={'time_s','raw_rad_s','clean_rad_s','osc_rad_s','fFiltered_Hz','valid', ...
    'phasePred_rad','phaseNext_rad','modelVersion','trainMaxIndex','memoryCount', ...
    'state','appliedCorrection_rad','fRaw_Hz','latentVariance','phaseClock_rad'};
writetable(array2table(algorithm.log,'VariableNames',names),fullfile(folder,'algorithm_trace.csv'));
writetable(array2table(algorithm.fft,'VariableNames', ...
    {'time_s','frequency_Hz','peakRatio','accepted'}),fullfile(folder,'fft_log.csv'));
cn={'time_s','requested_rad','applied_rad','coefficient','allowed','limit_flag', ...
    'lag_s','phaseBefore_rad','phaseAfter_rad','blockFirstIndex','blockLastIndex'};
writetable(array2table(algorithm.correction,'VariableNames',cn),fullfile(folder,'phase_correction.csv'));
if c.makePlots, plotResults(OUT,c); end
fprintf('Saved: %s\nEvaluation window %.3f to %.3f s\n',folder,window);
disp(metrics(:,{'motor','quantity','baselineBandRMS','algorithmBandRMS','bandReduction_pct'}));
end

function r=runChannel(useAlgorithm,t,sensor,wing,c)
% All exogenous inputs are shared; integrators/motor/delay states evolve per run.
clear flappy_quad_controller flappy_quad_actuator
N=numel(t); r.controller=zeros(N,24); r.actuator=zeros(N,48);
r.feedbackY=zeros(N,1); r.log=nan(N,16); r.fft=zeros(0,4); r.correction=zeros(0,11);
state=gyroY_online_init(); previousAct=zeros(48,1);
for k=1:N
    feedback=sensor(k,:).';
    if useAlgorithm
        [state,o]=gyroY_online_step(state,t(k),feedback(13),c);
        feedback(13)=o.clean;
        r.log(k,:)=[t(k) o.raw o.clean o.osc o.fFiltered o.valid o.phasePred ...
            o.phaseNext o.modelVersion o.trainMaxIndex o.memoryCount o.state ...
            o.appliedCorrection o.fRaw o.latentVariance o.phaseClock];
        if ~isempty(o.fft), r.fft(end+1,:)=o.fft; end
        if ~isempty(o.correction), r.correction(end+1,:)=o.correction; end
    end
    control=flappy_quad_controller([feedback;wing(k,:).';previousAct]);
    act=flappy_quad_actuator([t(k);control]);
    r.controller(k,:)=control.'; r.actuator(k,:)=act.';
    r.feedbackY(k)=feedback(13); previousAct=act;
    if mod(k,500)==0, drawnow limitrate nocallbacks; end
end
end

function metrics=scoreMotors(t,a,b,window,f0,c)
% Band RMS is signal activity, NOT energy or power consumption.
mask=t>=window(1) & t<window(2); fs=1/median(diff(t));
kind=["throttle","rpm","thrust_N"]; columns=[16 12 24]; scale=[1 60/(2*pi) 1];
rows=cell(12,11); k=0;
for motor=1:4
    for j=1:3
        x=a(mask,columns(j)+motor)*scale(j); y=b(mask,columns(j)+motor)*scale(j);
        if ~(window(2)-window(1)>=1)
            k=k+1; rows(k,:)=[{motor,kind(j)},num2cell(nan(1,9))];
            continue;
        end
        [f,px]=localPSD(x,fs); [~,py]=localPSD(y,fs); band=false(size(f));
        for h=1:c.nHarm, band=band | abs(f-h*f0)<=c.harmonicBandHalfWidth; end
        bx=sqrt(sum(px(band))*(f(2)-f(1))); by=sqrt(sum(py(band))*(f(2)-f(1)));
        k=k+1;
        rows(k,:)={motor,kind(j),mean(x),mean(y),sqrt(mean((x-mean(x)).^2)), ...
            sqrt(mean((y-mean(y)).^2)),max(x)-min(x),max(y)-min(y),bx,by,100*(1-by/bx)};
    end
end
metrics=cell2table(rows,'VariableNames',{'motor','quantity','baselineMean','algorithmMean', ...
    'baselineACRMS','algorithmACRMS','baselinePeakToPeak','algorithmPeakToPeak', ...
    'baselineBandRMS','algorithmBandRMS','bandReduction_pct'});
end

function [f,p]=localPSD(x,fs)
% Single-window one-sided periodogram; no toolbox, no claimed energy metric.
x=x(:); n=numel(x); w=0.5-0.5*cos(2*pi*(0:n-1)'/(n-1));
z=fft((x-mean(x)).*w); p=abs(z(1:floor(n/2)+1)).^2/(fs*sum(w.^2));
if mod(n,2)==0, p(2:end-1)=2*p(2:end-1); else, p(2:end)=2*p(2:end); end
f=(0:floor(n/2))'*fs/n;
end

function writeChannel(r,t,file)
d=table(t,'VariableNames',{'time_s'});
for j=1:4
    n=['M' num2str(j)];
    d.([n '_throttle'])=r.actuator(:,16+j);
    d.([n '_delayed_throttle'])=r.actuator(:,20+j);
    d.([n '_rpm'])=r.actuator(:,12+j)*60/(2*pi);
    d.([n '_thrust_N'])=r.actuator(:,24+j);
    d.([n '_reaction_Nm'])=r.actuator(:,28+j);
    d.([n '_equivalent_current_A'])=r.actuator(:,32+j);
end
d.gyroY_feedback_rad_s=r.feedbackY;
d.pitch_torque_cmd_Nm=r.controller(:,3);d.pitch_torque_actual_Nm=r.actuator(:,3);
d.allocation_saturated=r.actuator(:,48);
writetable(d,file);
end

function plotResults(R,c)
t=R.t; window=[max(t(1),t(end)-0.5),t(end)];
f=figure('Visible',c.figureVisible,'Color','w','Position',[100 100 1250 900]);
tiledlayout(4,3);
kind={'Throttle','Speed (rpm)','Thrust (N)'}; columns=[16 12 24]; scale=[1 60/(2*pi) 1];
for i=1:4
    for j=1:3
        nexttile;plot(t,R.baseline.actuator(:,columns(j)+i)*scale(j),'Color',[.3 .3 .3]);hold on;
        plot(t,R.algorithm.actuator(:,columns(j)+i)*scale(j),'b');grid on;xlim(window);
        title(sprintf('M%d %s',i,kind{j}));xlabel('Time (s)');
        if i==1 && j==1,legend('Baseline','Algorithm');end
    end
end
saveas(f,fullfile(R.outputFolder,'motor_zoom.png'));
f=figure('Visible',c.figureVisible,'Color','w','Position',[100 100 1100 750]);
tiledlayout(3,1);
nexttile;plot(t,R.sensorInput(:,13),'Color',[.3 .3 .3]);hold on;
plot(t,R.algorithm.feedbackY,'b');grid on;xlim(window);ylabel('Gyro Y (rad/s)');legend('Raw','Clean');
nexttile;plot(t,R.algorithm.log(:,5));grid on;ylabel('Frequency (Hz)');
nexttile;plot(t,R.baseline.controller(:,3),'Color',[.3 .3 .3]);hold on;
plot(t,R.algorithm.controller(:,3),'b');grid on;xlim(window);ylabel('Pitch torque (N m)');xlabel('Time (s)');
saveas(f,fullfile(R.outputFolder,'gyro_and_torque.png'));
end

function restoreConfig(previousCfg)
global FLAPPY_SIL_RUNTIME_CONFIG
FLAPPY_SIL_RUNTIME_CONFIG=previousCfg;
clear flappy_quad_controller flappy_quad_actuator
end
