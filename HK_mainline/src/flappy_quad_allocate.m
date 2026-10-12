function [thrust,achieved,throttle]=flappy_quad_allocate(request,q)
% Exact bounded allocation for symmetric square X rotors.
% Fix collective to the physically clipped height-loop request, then
% minimise roll/pitch error, then yaw error. Attitude allocation must NEVER
% increase collective to achieve an infeasible torque request.
% Opposite-pair differences diagonalize the equal-weight RP objective.
F=q.thrust_max_N; a=q.arm_xy_m;
collective=min(max(request(1),0),4*F);
dx=request(2)/a; dy=-request(3)/a;
desired=[(dx+dy)/2;(-dx+dy)/2];
% Feasible opposite-pair differences satisfy |u|,|v|<=F and
% |u|+|v|<=min(collective,4F-collective). Euclidean projection in this
% plane gives the least-squares roll/pitch solution at fixed collective.
magnitude=abs(desired);limited=min(magnitude,F);
budget=min(collective,4*F-collective);
if sum(limited)>budget
    % Active diamond boundary: x+y=budget, with 0<=x,y<=F.
    % Project the ORIGINAL request (not its box-clipped version).
    x=min(max((budget+magnitude(1)-magnitude(2))/2, ...
        max(0,budget-F)),min(F,budget));
    limited=[x;budget-x];
end
difference=sign(desired).*limited;
u=difference(1);v=difference(2);
base=abs(u)+abs(v);
H=(collective-base)/2;
lo=max(0,H-(F-abs(v))); hi=max(lo,min(F-abs(u),H));
h1=min(max((2*H+abs(v)-abs(u)-request(4)/(q.kQ/q.kT))/4,lo),hi);
h2=H-h1;
thrust=[h1+max(u,0);h2+max(v,0);h1+max(-u,0);h2+max(-v,0)];
thrust=min(max(thrust,0),F);
achieved=q.B*thrust;
w=sqrt(thrust/q.kT);
throttle=(q.resistance_equivalent_ohm*(q.no_load_current_A+q.kQ*w.^2/q.kt)+q.ke*w)/q.voltage_V;
throttle(thrust<1e-12)=0;
throttle=min(max(throttle,0),1);
end
