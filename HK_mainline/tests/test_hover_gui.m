function report=test_hover_gui()
root=startup_mainline();p=hk_hover_defaults();
% A sensor error must not silently change the selected control gains.
p.wing_enabled=true;p.left_wing_pct=3;p.right_wing_pct=-3;p.noise_enabled=true;
p.xy_kp_x=2.1;p.xy_kd_y=2.3;p.z_vel_kp=3.2;p.motor_delay_ms=11;p.sensor_delay_ms=3;
[c,info]=hk_hover_config(p,root);
assert(isequal(c.disturbance.position_kp_s2,c.position_kp_s2));
assert(c.disturbance.height_velocity_pid_kp==p.z_vel_kp);
assert(c.disturbance.left_aero_scale==1.03&&c.disturbance.right_aero_scale==.97);
assert(info.motor_delay_ms==12&&c.sensor_delay_samples==2);
% Invalid imported data must fail before modifying UI or loading a model.
bad=p;bad.roll_kp=NaN;mustReject(bad);bad=p;bad.extra=1;mustReject(bad);
bad=p;bad.wing_enabled=.5;mustReject(bad);bad=p;bad.seed=.5;mustReject(bad);
f=fullfile(root,'validation','gui_parameters_roundtrip.json');hk_hover_write_json(f,p);
assert(isequal(hk_hover_validate(jsondecode(fileread(f))),p));
% Actual controller and wing kernel observe selected gains/scales.
global FLAPPY_SIL_RUNTIME_CONFIG
prior=FLAPPY_SIL_RUNTIME_CONFIG;clean=onCleanup(@()restore(prior));
FLAPPY_SIL_RUNTIME_CONFIG=c;clear flappy_sil_wing_loads
input=[.4;15;1;0;0;0;zeros(6,1)];biased=flappy_sil_wing_loads(input);
c.disturbance.enable_wing_mismatch=false;FLAPPY_SIL_RUNTIME_CONFIG=c;
nominal=flappy_sil_wing_loads(input);
assert(norm(biased(3:5)-1.03*nominal(3:5))<1e-10);
assert(norm(biased(6:8)-.97*nominal(6:8))<1e-10);
% Native UI construction/parameter round-trip without simulating.
app=hk_hover_gui(Visible="off");uiCleanup=onCleanup(@()delete(app));
app.setParameters(p);assert(isequal(app.getParameters(),p));
report=struct('passed',true,'checks','bounds, JSON, GUI, effective delays, controller gain selection, actual wing loads');
hk_hover_write_json(fullfile(root,'validation','gui_component_tests.json'),report);
end
function mustReject(p)
try,hk_hover_validate(p);catch e,assert(strcmp(e.identifier,'hk:parameters'));return;end
error('hk:test','Invalid parameters accepted.');
end
function restore(prior)
global FLAPPY_SIL_RUNTIME_CONFIG
FLAPPY_SIL_RUNTIME_CONFIG=prior;
end
