function result=run_hover(parameters)
%RUN_HOVER Headless equivalent of GUI; saved results have identical format.
arguments
 parameters (1,1) struct=struct()
end
startup_mainline();
if isempty(fieldnames(parameters)),parameters=hk_hover_defaults();end
session=HKHoverSession(parameters);cleanup=onCleanup(@()delete(session));
result=session.run();disp(result.metrics);
end
