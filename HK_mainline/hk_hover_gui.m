function app=hk_hover_gui(options)
%HK_HOVER_GUI Launch the native MATLAB parameter editor and hover simulator.
arguments
 options.Visible (1,1) string {mustBeMember(options.Visible,["on","off"])}="on"
end
startup_mainline();app=HKHoverApp(options.Visible);
end
