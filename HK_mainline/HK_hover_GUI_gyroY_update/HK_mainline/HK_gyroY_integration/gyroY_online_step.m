function [s,o]=gyroY_online_step(s,now,wy,c)
% One causal update of the original replay loop, with state made explicit.
% Input: monotonically sampled time [s], gyro Y [rad/s], original config.
% Output o.clean = wy - prediction when valid; otherwise original bypass.
% Prediction precedes inserting the current sample and precedes model update.
% FFT, clustering, GP and phase-correction helpers below are verbatim copies.
% No added clipping, smoothing, warm-up blending, or fallback estimator.

k=numel(s.t)+1;
s.t(k,1)=now; s.raw(k,1)=wy; s.osc(k,1)=NaN;
t=s.t; y=s.raw;
o=struct('t',now,'raw',wy,'fRaw',NaN,'fFiltered',NaN, ...
    'phaseClock',NaN,'phasePred',NaN,'phaseNext',NaN, ...
    'osc',NaN,'latentVariance',NaN,'clean',wy,'valid',false, ...
    'modelVersion',0,'trainMaxIndex',0,'memoryCount',0,'state',0, ...
    'fft',[],'correction',[],'modelUpdated',false,'appliedCorrection',0);

haveFrequency=isfinite(s.fFiltered);
if haveFrequency
    s.phaseClock=s.phaseClock+2*pi*s.fFiltered*(t(k)-t(k-1));
end
if k>=c.nFFT && (s.lastFFT==0 || k-s.lastFFT>=c.fftHop)
    idx=(k-c.nFFT+1):k;
    [fr,quality,accepted]=estimateFFT(t(idx),y(idx),c);
    s.lastFFT=k; o.fft=[now fr quality accepted];
    if accepted
        s.fRaw=fr; s.lastGoodFFT=now;
        if ~haveFrequency
            s.fFiltered=fr; s.phaseClock=0; s.offset=0;
        else
            a=1-exp(-(now-s.lastFTime)/c.freqTau);
            s.fFiltered=s.fFiltered+a*(fr-s.fFiltered);
        end
        s.lastFTime=now;
    end
end
if ~isfinite(s.fFiltered), return; end
o.fRaw=s.fRaw; o.fFiltered=s.fFiltered; o.phaseClock=s.phaseClock;
nowStale=now-s.lastGoodFFT>c.maxFreqStaleSeconds;
if nowStale && ~s.stale
    s.model=[]; s.mi=[]; s.mp=[]; s.mc=[]; s.blockIndices=[];
end
s.stale=nowStale;
if s.stale, o.state=1; return; end
phase=s.phaseClock+s.offset;
o.phasePred=phase; o.phaseNext=phase; o.state=1;
if ~isempty(s.model)
    assert(s.model.maxIndex<k,'Causality violation: current/future data used to predict.');
    [pred,pv]=gpPredict(s.model,mod(phase,2*pi));
    s.osc(k)=pred; o.osc=pred; o.latentVariance=pv; o.clean=y(k)-pred;
    o.valid=true; o.modelVersion=s.version; o.trainMaxIndex=s.model.maxIndex;
    o.state=3; s.blockIndices(end+1)=k;
end
s.mi(end+1)=k; s.mp(end+1)=mod(phase,2*pi); s.mc(end+1)=s.phaseClock;
keep=s.mc>=s.phaseClock-2*pi*c.memoryCycles;
s.mi=s.mi(keep); s.mp=s.mp(keep); s.mc=s.mc(keep);
if isempty(s.model)
    history=max(1,k-c.nFFT+1):k;
    histClock=s.phaseClock+2*pi*s.fFiltered*(t(history)-now);
    if numel(s.mi)==1
        take=histClock>=s.phaseClock-2*pi*c.memoryCycles;
        s.mi=history(take); s.mc=histClock(take)'; s.mp=mod(s.mc+s.offset,2*pi);
    end
    covered=s.mc(end)-s.mc(1)+2*pi*s.fFiltered*median(diff(t(history)));
    if covered>=2*pi*c.memoryCycles-1e-8
        s.model=buildModel(s.mp,y(s.mi),[],c,true,k); s.version=s.version+1;
        s.initIndex=k; s.blockStart=s.phaseClock; s.blockIndices=[];
        o.state=2; o.modelUpdated=true;
    end
elseif s.phaseClock-s.blockStart>=2*pi && numel(s.blockIndices)>=16
    ii=s.blockIndices;
    [delta,coef,~,~,best]=phaseCorrection(y(ii),s.osc(ii),t(ii),c);
    allowed=c.corrEnabled && coef>=c.corrMinCoefficient;
    applied=0;
    if allowed, applied=c.corrGain*max(-c.corrLimit,min(c.corrLimit,delta)); end
    o.correction=[now delta applied coef allowed abs(delta)>=c.corrLimit, ...
        best/(2*pi*s.fFiltered) phase phase+applied ii(1) ii(end)];
    s.offset=s.offset+applied; o.phaseNext=phase+applied; o.state=4;
    o.appliedCorrection=applied;
    s.model=buildModel(s.mp,y(s.mi),s.model.centers,c,false,k);
    s.version=s.version+1; o.modelUpdated=true;
    s.blockStart=s.phaseClock; s.blockIndices=[];
end
o.memoryCount=numel(s.mi);
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

