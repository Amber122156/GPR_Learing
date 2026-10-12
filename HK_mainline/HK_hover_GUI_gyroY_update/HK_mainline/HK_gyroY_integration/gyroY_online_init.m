function s=gyroY_online_init()
% Algorithm state only. One state per run; no persistent/global variables here.
% Growing t/raw/osc arrays retain original sample indexing for exact replay.
s.t=zeros(0,1); s.raw=zeros(0,1); s.osc=zeros(0,1);
s.fRaw=NaN; s.fFiltered=NaN; s.phaseClock=0; s.offset=0; s.model=[];
s.mi=[]; s.mp=[]; s.mc=[]; s.lastFFT=0; s.lastGoodFFT=-Inf;
s.lastFTime=NaN; s.version=0; s.stale=false;
s.blockStart=NaN; s.blockIndices=[]; s.initIndex=NaN;
end
