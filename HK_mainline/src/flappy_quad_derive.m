function q=flappy_quad_derive(q)
% Quan/Dai et al. Eq 10,12: geometric prior, NOT measured Gemfan data.
phi=atan(q.pitch_m/(0.75*pi*q.diameter_m));
q.CT=0.323*q.blades*phi*q.ct_multiplier;
q.CQ=0.0432*q.blades^2*(0.01+0.9*phi^2)*q.cq_multiplier;
q.kT=q.CT*q.rho*q.diameter_m^4/(2*pi)^2;
q.kQ=q.CQ*q.rho*q.diameter_m^5/(2*pi)^2;
% Eq 16 uses equivalent current/voltage; not ESC battery current.
q.ke=(q.no_load_voltage_V-q.no_load_current_A*q.resistance_equivalent_ohm)/ ...
    (q.kv_rpm_V*q.no_load_voltage_V)*60/(2*pi);
q.kt=q.ke;
assert(q.ke>0 && q.kT>0 && q.kQ>0);
q.omega_max=flappy_quad_steady(1,q);
q.thrust_max_N=q.kT*q.omega_max^2;
q.B=[ones(1,4);q.hubs_m(:,2).';-q.hubs_m(:,1).'; ...
    q.reaction_sign.'*(q.kQ/q.kT)]; % maps four positive thrusts to wrench
end
