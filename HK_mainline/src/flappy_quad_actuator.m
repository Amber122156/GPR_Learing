function out=flappy_quad_actuator(input)
% Input [time; legacy24 controller]. Output 48:
% legacy12; omega4; throttle4; delayedThrottle4; T4; signedQ4;
% equivalentCurrent4; allocatedWrench4; requestWrench4; actualCollective;
% estimatedWingFz; projection; allocationSaturated.
persistent buffer omega
cfg=flappy_sil_active_config(); q=cfg.quad; dt=cfg.control_sample_time_s;
t=input(1); c=input(2:25);
request=[c(20);c(2:4)];
[~,allocated,u]=flappy_quad_allocate(request,q);
mode="dynamic";
if isfield(q,'actuator_mode'),mode=string(q.actuator_mode);end
if isempty(buffer)||t<=1e-12
    buffer=zeros(max(0,round(q.delay_s/dt)),4); omega=zeros(4,1);
end
request=[c(20);c(2:4)];
[~,allocated,u]=flappy_quad_allocate(request,q);
if mode=="ideal"
    % Analysis mode: allocation and physical thrust limits are retained, but
    % the commanded rotor equilibrium is reached in the current sample.
    delayed=u;
    omega=flappy_quad_steady(delayed,q);
else
    if isempty(buffer)
        delayed=u;
    else
        delayed=buffer(1,:).'; buffer=[buffer(2:end,:);u.'];
    end
    targetOmega=flappy_quad_steady(delayed,q);
    if q.time_constant_s<=eps
        omega=targetOmega;
    else
        omega=omega+(1-exp(-dt/q.time_constant_s))*(targetOmega-omega);
    end
end
T=q.kT*omega.^2; Q=q.reaction_sign.*q.kQ.*omega.^2;
actual=q.B*T;
res=allocated-request;
sat=abs(res(2:4))>1e-5;
I=q.no_load_current_A+q.kQ*omega.^2/q.kt; I(omega<1e-9)=0;
out=[c(1);actual(2:4);c(1);allocated(2:4);double(sat);double(abs(res(1))>1e-4); ...
 omega;u;delayed;T;Q;I;allocated;request;actual(1);c(24)-c(20)*c(21);c(21);double(any(abs(res)>1e-4))];
assert(numel(out)==48);
end
