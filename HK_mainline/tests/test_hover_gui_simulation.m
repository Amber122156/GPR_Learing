function report=test_hover_gui_simulation()
% Explicit integration test; two 5 s flights plus stop/save test.
root=startup_mainline();
global FLAPPY_SIL_RUNTIME_CONFIG
prior=FLAPPY_SIL_RUNTIME_CONFIG;
app=hk_hover_gui(Visible="off");cleanup=onCleanup(@()delete(app));
p=hk_hover_defaults();p.duration_s=5;app.setParameters(p);app.startSimulation();
wall=tic;while app.Running&&toc(wall)<600,pause(.2);end
assert(~app.Running,'Timed out');assert(strlength(app.LastError)==0,char(app.LastError));
baseline=app.Result;assert(baseline.run_status=="completed"&&baseline.metrics.finite&&baseline.metrics.valid_window);
assert(baseline.metrics.pass);disp(baseline.metrics);
app.Figure.Visible='on';drawnow;exportapp(app.Figure,fullfile(baseline.config.project_root,'validation','hover_gui_preview.png'));app.Figure.Visible='off';
p.height_m=1.0;p.z_vel_kp=5;p.wing_enabled=true;p.left_wing_pct=3;p.right_wing_pct=-3;
p.motor_delay_ms=20;p.motor_tau_ms=40;p.sensor_delay_ms=6;p.noise_enabled=true;
app.setParameters(p);app.startSimulation();wall=tic;
while app.Running&&toc(wall)<600,pause(.2);end
assert(~app.Running&&strlength(app.LastError)==0,char(app.LastError));changed=app.Result;
assert(changed.metrics.finite&&changed.run_status=="completed");
assert(changed.applied_info.motor_delay_ms==20&&changed.applied_info.sensor_delay_ms==6);
assert(changed.config.disturbance.height_velocity_pid_kp==5);
assert(max(abs(changed.diagnostics.z_m-baseline.diagnostics.z_m))>.02);
% Manually stop a long flight; reported data must end where the model stopped.
p=hk_hover_defaults();p.duration_s=120;app.setParameters(p);app.startSimulation();wall=tic;requested=false;
while app.Running&&toc(wall)<600
 pause(.2);
 if ~requested&&~isempty(app.Session)
  [done,t]=app.Session.poll();
  if ~done&&t>.2,app.stopSimulation();requested=true;end
 end
end
assert(requested&&~app.Running&&strlength(app.LastError)==0,char(app.LastError));stopped=app.Result;
assert(stopped.run_status=="stopped_by_user");assert(stopped.diagnostics.time_s(end)<120);
assert(abs(stopped.diagnostics.time_s(end)-stopped.config.duration_s)<1e-9);
global FLAPPY_SIL_RUNTIME_CONFIG
assert(isequal(FLAPPY_SIL_RUNTIME_CONFIG,prior));assert(~bdIsLoaded('flappy_hk_gui_hover'));
report=struct('passed',true,'baseline_metrics',baseline.metrics,'changed_metrics',changed.metrics, ...
 'baseline_folder',baseline.output_folder,'changed_folder',changed.output_folder, ...
 'stopped_duration_s',stopped.config.duration_s,'stop_folder',stopped.output_folder, ...
 'checks','native UI, timer start/stop, completed baseline, changed gains/wing/delay/noise, no extrapolation, cleanup');
hk_hover_write_json(fullfile(baseline.config.project_root,'validation','gui_simulation_tests.json'),report);
disp('GUI_ACTUAL_SIMULATION_TESTS_PASSED');

end
