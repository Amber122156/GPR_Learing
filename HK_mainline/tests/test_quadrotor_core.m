function report=test_quadrotor_core()
root=string(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(root,'src'));
c=flappy_quad_config(root);q=c.quad;
rigid=flappy_hk_rigid_parameters();
assert(abs(c.mass_kg-rigid.total_mass_kg)<1e-10);
assert(abs(norm(q.hubs_m(1,:)-q.hubs_m(3,:))-0.180)<1e-12);
assert(all(abs(q.hubs_m(:,3)+0.0510087)<1e-12));
u=linspace(0,1,101);[w,I]=flappy_quad_steady(u,q);
assert(all(diff(w)>=0)&&w(1)==0);
mask=u>=q.deadzone;
assert(max(abs(q.resistance_equivalent_ohm*I(mask)+q.ke*w(mask)-u(mask)*q.voltage_V))<1e-10);
assert(q.omega_max<q.kv_rpm_V*q.voltage_V*2*pi/60);
request=[1.5;0.01;-0.015;0.002];
[F,got,throttle]=flappy_quad_allocate(request,q);
assert(max(abs(got-request))<1e-10);
assert(max(abs(q.kT*flappy_quad_steady(throttle,q).^2-F))<1e-10);
% Independent cross-product wrench verifies signs at the physical hubs.
tau=zeros(3,1);
for i=1:4,tau=tau+cross(q.hubs_m(i,:).',[0;0;F(i)])+[0;0;q.reaction_sign(i)*q.kQ/q.kT*F(i)];end
assert(norm(tau-request(2:4))<1e-10);
for axis=2:4
 for sign=[-1 1]
  r=[1.5;0;0;0];r(axis)=sign*0.005;
  [~,g]=flappy_quad_allocate(r,q);assert(sign*g(axis)>0);
 end
end
[F,~]=flappy_quad_allocate([100;10;-10;5],q);
assert(all(F>=0 & F<=q.thrust_max_N));
% Large attitude demands may sacrifice torque, NEVER the requested total
% thrust. Verify RP optimality against an independent polygon projection.
stream=RandStream('mt19937ar','Seed',20260929);
for k=1:500
 r=[(6*rand(stream)-1)*q.thrust_max_N;randn(stream,3,1)];
 [F,g,throttle]=flappy_quad_allocate(r,q);
 total=min(max(r(1),0),4*q.thrust_max_N);
 assert(abs(g(1)-total)<1e-10);
 assert(all(F>=0 & F<=q.thrust_max_N+1e-12));
 actualF=q.kT*flappy_quad_steady(throttle,q).^2;
 % The physical startup deadzone can only REDUCE a tiny allocated thrust.
 activeMotor=throttle>=q.deadzone;
 assert(all(abs(actualF(activeMotor)-F(activeMotor))<1e-10));
 assert(all(actualF<=F+1e-10));
 desired=[(r(2)-r(3))/(2*q.arm_xy_m);(-r(2)-r(3))/(2*q.arm_xy_m)];
 reference=localProjectPolygon(desired,q.thrust_max_N,min(total,4*q.thrust_max_N-total));
 assert(norm([F(1)-F(3);F(2)-F(4)]-reference)<1e-9);
end
for total=[0,4*q.thrust_max_N]
 [~,g]=flappy_quad_allocate([total;10;-10;5],q);
 assert(abs(g(1)-total)<1e-10 && norm(g(2:4))<1e-10);
end
% Constant command: exact delay then 63.2% of final rotor speed after 20 ms.
global FLAPPY_SIL_RUNTIME_CONFIG
FLAPPY_SIL_RUNTIME_CONFIG=c;cleanup=onCleanup(@reset);
clear flappy_quad_actuator
cmd=zeros(24,1);cmd(1)=20;cmd(20)=1.5;cmd(21)=1;
for k=0:14
 out=flappy_quad_actuator([k*0.002;cmd]);
 if k<5,assert(all(out(13:16)==0));end
end
[~,~,u]=flappy_quad_allocate([1.5;0;0;0],q);
assert(max(abs(out(13:16)./flappy_quad_steady(u,q)-(1-exp(-1))))<1e-12);
report=struct('passed',true,'mass_kg',c.mass_kg,'weight_N',c.weight_N, ...
 'wing20_N',q.nominal_wing_lift_N,'rotor_hover_total_N',c.weight_N-q.nominal_wing_lift_N, ...
 'estimated_single_max_N',q.thrust_max_N,'estimated_max_rpm',q.omega_max*60/(2*pi));
disp(report);
end
function projection=localProjectPolygon(desired,F,budget)
% Independent reference: closest point on the feasible polygon's edges.
if budget==0,projection=[0;0];return;end
if budget<=F
 vertices=[budget 0;0 budget;-budget 0;0 -budget];
else
 s=budget-F;
 vertices=[F s;s F;-s F;-F s;-F -s;-s -F;s -F;F -s];
end
if all(abs(desired)<=F)&&sum(abs(desired))<=budget,projection=desired;return;end
best=inf;projection=zeros(2,1);
for i=1:size(vertices,1)
 start=vertices(i,:)';finish=vertices(mod(i,size(vertices,1))+1,:)';edge=finish-start;
 if dot(edge,edge)==0,candidate=start;
 else,candidate=start+min(max(dot(desired-start,edge)/dot(edge,edge),0),1)*edge;end
 cost=sum((candidate-desired).^2);
 if cost<best,best=cost;projection=candidate;end
end
end
function reset()
global FLAPPY_SIL_RUNTIME_CONFIG
FLAPPY_SIL_RUNTIME_CONFIG=[];
end
