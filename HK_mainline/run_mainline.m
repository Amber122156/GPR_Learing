function result=run_mainline(caseName,options)
%RUN_MAINLINE Simplified GUI / takeoff-hover entry. Historical cases are archived.
arguments
 caseName (1,1) string {mustBeMember(caseName,["gui","hover","smoke","validate"])}="gui"
 options.Parameters (1,1) struct=struct()
 options.Duration double=[]
 options.Seed double=[]
end
startup_mainline();p=options.Parameters;
if isempty(fieldnames(p)),p=hk_hover_defaults();end
if ~isempty(options.Duration),p.duration_s=options.Duration;end
if ~isempty(options.Seed),p.seed=options.Seed;end
p=hk_hover_validate(p);
switch caseName
 case "gui",result=hk_hover_gui();result.setParameters(p);
 case "hover",result=run_hover(p);
 case "smoke",result=smoke_mainline();
 case "validate",result=validate_mainline();
end
end
