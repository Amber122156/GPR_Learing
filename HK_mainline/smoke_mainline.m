function report=smoke_mainline()
%SMOKE_MAINLINE Check the actual GUI-hover plant, including zero motor-delay setting.
p=hk_hover_defaults();p.duration_s=1;p.motor_delay_ms=0;
r=run_hover(p);assert(r.metrics.finite&&r.run_status=="completed");
report=struct('passed',true,'model','flappy_hk_gui_hover','duration_s',1, ...
 'scope','compile/runtime smoke only; not hover stability acceptance');
hk_hover_write_json(fullfile(r.config.project_root,'validation','smoke_report.json'),report);
disp('HOVER_SMOKE_PASSED');
end
