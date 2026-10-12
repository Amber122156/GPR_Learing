function report=verify_gyroY_step(baselineFile,userCfg)
% Compare step module against the original replay loop on the SAME input.
% The original loop and helpers are included verbatim below, except its name.
% This is an algorithm-equivalence test, not a physical performance test.
if nargin<2,userCfg=struct;end
c=gyroY_online_config(userCfg);
loaded=load(baselineFile,'result');b=loaded.result;
stride=round(b.config.control_sample_time_s/b.config.plant_sample_time_s);
ix=(1:stride:height(b.diagnostics)).';
t=b.diagnostics.time_s(ix); y=b.sensor(ix,13);
reference=originalReplay(t,y,c);
names={'raw','fRaw','fFiltered','phaseClock','phasePred','phaseNext','osc', ...
    'latentVariance','clean','valid','modelVersion','trainMaxIndex','memoryCount','state'};
got=zeros(numel(t),numel(names)); state=gyroY_online_init();ff=zeros(0,4);pc=zeros(0,11);
for k=1:numel(t)
    [state,o]=gyroY_online_step(state,t(k),y(k),c);
    for j=1:numel(names),got(k,j)=o.(names{j});end
    if ~isempty(o.fft),ff(end+1,:)=o.fft;end
    if ~isempty(o.correction),pc(end+1,:)=o.correction;end
end
rows=cell(numel(names)+2,3);
for j=1:numel(names)
    [e,pass]=compare(reference.(names{j}),got(:,j));
    rows(j,:)={names{j},e,pass};
end
[e,pass]=compare(reference.fft,ff); rows(end-1,:)={'fft_log',e,pass};
[e,pass]=compare(reference.correction,pc);rows(end,:)={'phase_correction',e,pass};
report=cell2table(rows,'VariableNames',{'quantity','maxAbsError','pass'});
disp(report);assert(all(report.pass),'Step algorithm differs from original replay.');
fprintf('All sample outputs, FFT decisions and phase corrections match.\n');
end

function [e,pass]=compare(a,b)
sameMask=isequal(isnan(a),isnan(b)) && isequal(isinf(a),isinf(b));
finite=isfinite(a)&isfinite(b);e=max([0;abs(a(finite)-b(finite))]);
pass=sameMask && e<1e-10;
end

function R=originalReplay(t,y,c)
N=numel(t); nanN=nan(N,1);
R=struct('t',t,'raw',y,'fRaw',nanN,'fFiltered',nanN,'phaseClock',nanN,...
 'phasePred',nanN,'phaseNext',nanN,'osc',nanN,'latentVariance',nanN,...
 'clean',y,'valid',false(N,1),'modelVersion',zeros(N,1),...
 'trainMaxIndex',zeros(N,1),'memoryCount',zeros(N,1),'state',zeros(N,1));
R.fft=[]; R.spectrum=[]; R.spectrumFrequency=[]; R.events=cell(0,5);
R.correction=[]; R.corrDetail=struct('t',{},'lagRad',{},'R',{});
R.models=[]; R.snapshots=struct('t',{},'phi',{},'y',{},'indices',{},'phaseClock',{},'model',{});
fRaw=NaN; fFiltered=NaN; phaseClock=0; offset=0; model=[];
mi=[]; mp=[]; mc=[]; lastFFT=0; lastGoodFFT=-Inf;
lastFTime=NaN; lastSnapshot=-Inf; version=0; stale=false;
blockStart=NaN; blockIndices=[]; initIndex=NaN; modelDirty=false;
for k=1:N
    now=t(k); haveFrequency=isfinite(fFiltered);
    % 已知上一时刻频率推进当前相位；不将本次新FFT追溯应用到过去。
    if haveFrequency, phaseClock=phaseClock+2*pi*fFiltered*(t(k)-t(k-1)); end
    if k>=c.nFFT && (lastFFT==0 || k-lastFFT>=c.fftHop)
        idx=(k-c.nFFT+1):k;
        [fr,quality,accepted,power,faxis]=estimateFFT(t(idx),y(idx),c);
        lastFFT=k; R.fft(end+1,:)=[now fr quality accepted];
        R.spectrum(:,end+1)=power; R.spectrumFrequency=faxis;
        if accepted
            fRaw=fr; lastGoodFFT=now;
            if ~haveFrequency
                fFiltered=fr; phaseClock=0; offset=0;
                R.events(end+1,:)={now,k,'frequency_ready',k,'initial FFT accepted'};
            else
                a=1-exp(-(now-lastFTime)/c.freqTau);
                fFiltered=fFiltered+a*(fr-fFiltered);
            end
            lastFTime=now;
        end
    end
    if ~isfinite(fFiltered), continue; end
    R.fRaw(k)=fRaw; R.fFiltered(k)=fFiltered; R.phaseClock(k)=phaseClock;
    nowStale=now-lastGoodFFT>c.maxFreqStaleSeconds;
    if nowStale && ~stale
        model=[]; mi=[]; mp=[]; mc=[]; blockIndices=[];
        R.events(end+1,:)={now,k,'frequency_stale',k,'bypass, discard model and memory'};
    elseif ~nowStale && stale
        R.events(end+1,:)={now,k,'frequency_recovered',k,'reinitialize from available history'};
    end
    stale=nowStale;
    if stale, R.state(k)=1; continue; end
    phase=phaseClock+offset;
    R.phasePred(k)=phase; R.phaseNext(k)=phase; R.state(k)=1;
    % 先预测；model严格来自k-1或更早，当前测量还没有进入训练集。
    if ~isempty(model)
        assert(model.maxIndex<k,'Causality violation: current/future data used to predict.');
        [pred,pv]=gpPredict(model,mod(phase,2*pi));
        R.osc(k)=pred; R.latentVariance(k)=pv; R.clean(k)=y(k)-pred;
        R.valid(k)=true; R.modelVersion(k)=version; R.trainMaxIndex(k)=model.maxIndex;
        R.state(k)=3; blockIndices(end+1)=k;
    end
    % 相位标签一旦产生即固定；之后的校正只改变未来phase，不事后移动历史点。
    mi(end+1)=k; mp(end+1)=mod(phase,2*pi); mc(end+1)=phaseClock;
    keep=mc>=phaseClock-2*pi*c.memoryCycles;
    mi=mi(keep); mp=mp(keep); mc=mc(keep);
    if isempty(model)
        % 一次性历史初始化：仅用截至当前的FFT历史窗口。constant-f回填是近似。
        history=max(1,k-c.nFFT+1):k;
        histClock=phaseClock+2*pi*fFiltered*(t(history)-now);
        if numel(mi)==1
            take=histClock>=phaseClock-2*pi*c.memoryCycles;
            mi=history(take); mc=histClock(take)'; mp=mod(mc+offset,2*pi);
        end
        covered=mc(end)-mc(1)+2*pi*fFiltered*median(diff(t(history)));
        if covered>=2*pi*c.memoryCycles-1e-8
            model=buildModel(mp,y(mi),[],c,true,k); version=version+1;
            initIndex=k; blockStart=phaseClock; blockIndices=[];
            R.events(end+1,:)={now,k,'memory_ready',k,'historical phase backfill plus causal accumulation'};
            R.events(end+1,:)={now,k,'initial_gpr_ready',k,'prediction begins on NEXT sample'};
            R.state(k)=2; modelDirty=true;
        end
    elseif phaseClock-blockStart>=2*pi && numel(blockIndices)>=16
        % 模型在此周期内冻结；该周期全部为更新前真实在线预测。
        ii=blockIndices; [delta,coef,lags,cc,best]=phaseCorrection(R.raw(ii),R.osc(ii),t(ii),c);
        allowed=c.corrEnabled && coef>=c.corrMinCoefficient;
        applied=0;
        if allowed, applied=c.corrGain*max(-c.corrLimit,min(c.corrLimit,delta)); end
        R.correction(end+1,:)=[now delta applied coef allowed abs(delta)>=c.corrLimit,...
            best/(2*pi*fFiltered) phase phase+applied ii(1) ii(end)];
        R.corrDetail(end+1)=struct('t',now,'lagRad',lags,'R',cc);
        offset=offset+applied; R.phaseNext(k)=phase+applied; R.state(k)=4;
        R.events(end+1,:)={now,k,'phase_correction',max(ii),sprintf('applied %.6g rad',applied)};
        % 训练仅影响下一采样点。允许一个周期更新一次统计量与GP。
        model=buildModel(mp,y(mi),model.centers,c,false,k); version=version+1;
        modelDirty=true; blockStart=phaseClock; blockIndices=[];
    end
    R.memoryCount(k)=numel(mi);
    if modelDirty
        R.models(end+1,:)=[now version model.maxIndex numel(mi) min(model.count),...
            model.gapMin model.gapMax model.recluster model.jitter model.condition];
        R.events(end+1,:)={now,k,'model_update',model.maxIndex,sprintf('version %d',version)};
        if now-lastSnapshot>=c.snapshotSeconds || numel(R.snapshots)==0
            R.snapshots(end+1)=struct('t',now,'phi',mp(:),'y',y(mi),'indices',mi(:),'phaseClock',mc(:),'model',model);
            lastSnapshot=now;
        end
        modelDirty=false;
    end
end
R.initIndex=initIndex;
if ~isempty(model)
    R.snapshots(end+1)=struct('t',t(end),'phi',mp(:),'y',y(mi),'indices',mi(:),'phaseClock',mc(:),'model',model);
end
assert(~any(R.trainMaxIndex(R.valid)>=find(R.valid)),'Noncausal prediction.');
end

function [fr,q,ok,p,f]=estimateFFT(t,y,c)
n=numel(y); fs=1/median(diff(t));
w=0.5-0.5*cos(2*pi*(0:n-1)'/(n-1));
z=fft((y-mean(y)).*w); p=abs(z(1:n/2+1)).^2/(fs*sum(w.^2));
p(2:end-1)=2*p(2:end-1); f=(0:n/2)'*fs/n;
bins=find(f>=c.freqBand(1) & f<=c.freqBand(2));
assert(numel(bins)>=3,'Frequency search band needs at least 3 FFT bins.');
[peak,j]=max(p(bins)); b=bins(j); q=peak/max(median(p(bins)),eps);
fr=f(b); ok=q>=c.minPeakRatio && j>1 && j<numel(bins);
if b>1 && b<numel(p)
    a=log(max(p(b-1:b+1),realmin)); den=a(1)-2*a(2)+a(3);
    if abs(den)>eps
        shift=max(-0.5,min(0.5,0.5*(a(1)-a(3))/den)); fr=fr+shift*fs/n;
    end
end
% 插值改善峰位置估计，但不会改变原始FFT分辨率fs/n。
end

function model=buildModel(phi,y,centers,c,initial,maxIndex)
phi=phi(:); y=y(:);
if initial
    start=(0:c.K-1)'*2*pi/c.K;
    opt=optimset('Display','off','MaxIter',c.initMaxIterations,...
        'MaxFunEvals',c.initMaxIterations*8,'TolX',1e-5,'TolFun',1e-7);
    centers=mod(fminsearch(@(x) sphereObjective(phi,x),start,opt),2*pi);
    centers=lloyd(phi,centers,c.kmeansIterations); recluster=true;
else
    % 每周期更新最近邻簇统计；重新求全局分配仅在中心聚集/空簇时触发。
    labels=assign(phi,centers); moved=centers;
    for j=1:c.K
        a=phi(labels==j); if ~isempty(a), moved(j)=circularMean(a); end
    end
    gaps=diff([sort(moved);min(moved)+2*pi]); ideal=2*pi/c.K;
    recluster=min(gaps)<c.gapMinRatio*ideal || max(gaps)>c.gapMaxRatio*ideal ||...
        numel(unique(labels))<c.K;
    centers=moved;
    if recluster, centers=lloyd(phi,centers,c.kmeansIterations); end
end
centers=sort(mod(centers,2*pi)); labels=assign(phi,centers);
mu=zeros(c.K,1); noise=mu; count=mu; phase=mu;
for j=1:c.K
    take=labels==j; count(j)=sum(take);
    assert(count(j)>0,'Empty cluster after reseeding.');
    phase(j)=circularMean(phi(take)); mu(j)=mean(y(take));
    if count(j)>1, noise(j)=var(y(take),0); else, noise(j)=var(y,0); end
end
[phase,order]=sort(phase); mu=mu(order); noise=noise(order); count=count(order);
% Eq(1): subtract mean of output observation vector (cluster means).
% 不把mean加回osc；傅里叶特征不含DC，周期模板积分严格为0。
target=mu-mean(mu); B=features(phase,c.nHarm);
K=2*c.kernelScale*(B*B'); noise=max(noise,c.noiseFloor);
% 使用簇内方差，不偷偷换成 variance/count（后者是另一噪声模型）。
jitter=c.jitter; A=K+diag(noise);
for attempt=1:10
    [L,flag]=chol(A+jitter*eye(c.K),'lower');
    if flag==0, break; end
    jitter=max(10*jitter,eps);
end
assert(flag==0,'GP covariance is not positive definite.');
alpha=L'\(L\target); gaps=diff([phase;phase(1)+2*pi]);
model=struct('centers',phase,'mean',mu,'target',target,'noise',noise,...
 'count',count,'B',B,'L',L,'alpha',alpha,'nHarm',c.nHarm,...
 'scale',c.kernelScale,'maxIndex',maxIndex,'jitter',jitter,...
 'condition',rcond(A+jitter*eye(c.K)),'recluster',recluster,...
 'gapMin',min(gaps)/(2*pi/c.K),'gapMax',max(gaps)/(2*pi/c.K));
end

function value=sphereObjective(phi,centers)
d=wrapPi(phi(:)-centers(:)'); value=sum(min(1-cos(d/2),[],2));
end
function labels=assign(phi,centers)
[~,labels]=min(1-cos(wrapPi(phi(:)-centers(:)')/2),[],2);
end
function centers=lloyd(phi,centers,maxIter)
for it=1:maxIter
    old=centers; labels=assign(phi,centers);
    for j=1:numel(centers)
        a=phi(labels==j);
        if isempty(a)
            distances=min(abs(wrapPi(phi(:)-centers(:)')),[],2);
            [~,ix]=max(distances); centers(j)=phi(ix);
        else
            centers(j)=circularMean(a);
        end
    end
    if max(abs(wrapPi(centers-old)))<1e-7, break; end
end
end
function x=circularMean(a)
x=mod(atan2(mean(sin(a)),mean(cos(a))),2*pi);
end
function x=wrapPi(x)
x=mod(x+pi,2*pi)-pi;
end
function B=features(phi,H)
a=phi(:)*(1:H); B=zeros(numel(phi),2*H);
B(:,1:2:end)=sin(a); B(:,2:2:end)=cos(a);
end
function [mu,pv]=gpPredict(m,phi)
B=features(phi,m.nHarm); cross=2*m.scale*(m.B*B');
mu=cross'*m.alpha; v=m.L\cross;
pv=max(0,2*m.scale*sum(B.^2,2)-sum(v.^2,1)');
end

function [delta,coef,lags,cc,best]=phaseCorrection(y,p,t,c)
% R(k)=sum y(i)*prediction(i+k). 正lag应使后续相位前进，selftest验证符号。
% 只对已完成新周期做圆周相关；不是对原文的双通道/双侧索引逐字复刻。
L=c.corrGrid; u=linspace(t(1),t(end),L+1)'; u(end)=[];
a=interp1(t,y,u,'linear'); b=interp1(t,p,u,'linear');
a=a-mean(a); b=b-mean(b); denominator=norm(a)*norm(b);
lag=(-floor(L/2):ceil(L/2)-1)'; cc=zeros(size(lag));
if denominator<=eps, delta=0; coef=0; lags=lag*2*pi/L; best=0; return; end
for j=1:numel(lag)
    shifted=b(mod((0:L-1)'+lag(j),L)+1); cc(j)=sum(a.*shifted)/denominator;
end
[coef,j]=max(cc); sub=0;
if j>1 && j<numel(cc)
    den=cc(j-1)-2*cc(j)+cc(j+1);
    if abs(den)>eps, sub=max(-0.5,min(0.5,0.5*(cc(j-1)-cc(j+1))/den)); end
end
best=2*pi*(lag(j)+sub)/L; delta=wrapPi(best); lags=lag*2*pi/L;
end

