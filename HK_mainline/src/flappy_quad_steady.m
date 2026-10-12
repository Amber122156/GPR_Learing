function [omega,current]=flappy_quad_steady(throttle,q)
% Loaded motor equilibrium: U=R*(I0+kQ*w^2/kt)+ke*w.
u=min(max(throttle,0),1);
c=max(u*q.voltage_V-q.resistance_equivalent_ohm*q.no_load_current_A,0);
a=q.resistance_equivalent_ohm*q.kQ/q.kt;
omega=2*c./(q.ke+sqrt(q.ke^2+4*a*c));
omega(u<q.deadzone)=0;
current=q.no_load_current_A+q.kQ*omega.^2/q.kt;
current(omega==0)=0;
end
