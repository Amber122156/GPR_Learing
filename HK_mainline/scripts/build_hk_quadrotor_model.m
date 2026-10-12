function modelName=build_hk_quadrotor_model()
% Build ONLY the new model from the existing verified disturbed baseline.
root=string(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'src'));addpath(fullfile(root,'scripts'));
cfg=flappy_quad_config(root); q=cfg.quad;
modelName="flappy_hk_quadrotor_position_hover";
source=fullfile(root,'models','flappy_hk_disturbed_position_hover.slx');
assert(isfile(source),'Build the existing disturbed baseline first.');
dest=fullfile(root,'models',modelName+'.slx');
if bdIsLoaded(modelName),close_system(modelName,0);end
copyfile(source,dest,'f');load_system(dest);
set_param(modelName,'StopTime','5','MaxStep','1e-4');
% Remove the virtual wrench itself and its conversion chain.
for n=["sil_virtual_torque_wrench","ps_sil_virtual_torque","sil_actual_torque_mux"]
 delete_block(modelName+'/'+n);
end
set_param(modelName+'/sil_virtual_actuator','MATLABFcn','flappy_quad_actuator(u)','OutputDimensions','48');
set_param(modelName+'/sil_actuator_demux','Outputs','[1 1 1 1 1 1 1 1 1 1 1 1 4 4 4 4 4 4 4 4 1 1 1 1]');
set_param(modelName+'/sil_hover_controller','MATLABFcn','flappy_quad_controller(u)');
lh=get_param(modelName+'/sil_hover_controller','LineHandles');delete_line(lh.Inport);
add_block('simulink/Signal Routing/Mux',modelName+'/quad_controller_inputs','Inputs','3','Position',[500 1100 510 1180]);
add_block('simulink/Discrete/Unit Delay',modelName+'/quad_feedback_delay', ...
 'InitialCondition','zeros(48,1)','SampleTime','0.002','Position',[350 1200 420 1240]);
wire('sil_sensor_delay',1,'quad_controller_inputs',1);
add_block('simulink/Discrete/Unit Delay',modelName+'/quad_wing_feedback_delay', ...
 'InitialCondition','zeros(28,1)','SampleTime','0.002','Position',[350 1110 420 1150]);
wire('sil_wing_loads',1,'quad_wing_feedback_delay',1);
wire('quad_wing_feedback_delay',1,'quad_controller_inputs',2);
wire('sil_virtual_actuator',1,'quad_feedback_delay',1);
wire('quad_feedback_delay',1,'quad_controller_inputs',3);
wire('quad_controller_inputs',1,'sil_hover_controller',1);
ground=flappy_ground_config(flappy_parameters(root));
for i=1:4
 local=[ground.support_xy_m(i,:),ground.torso_bottom_m-0.030];
 set_param(modelName+'/foot_'+ground.support_names(i)+'_frame','TranslationCartesianOffset',mat2str(local,12));
 leg=[local(1:2),local(3)+0.015];
 addVisual("quad_leg_"+i,leg,[0.004 0.004 0.030],[0.22 0.24 0.27],1);
end
set_param(modelName+'/torso_to_world','PzPositionTargetValue',num2str(cfg.initial_torso_z_m,14));
for i=1:4
 name="quad_M"+i;
 hub=modelName+'/'+name+'_hub';
 add_block('sm_lib/Frames and Transforms/Rigid Transform',hub, ...
 'TranslationMethod','Cartesian','TranslationCartesianOffset',mat2str(q.hubs_m(i,:),12), ...
 'Position',[1700 950+180*i 1760 1000+180*i]);
 hp=get_param(hub,'PortHandles');tp=get_param(modelName+'/torso','PortHandles');
 add_line(modelName,tp.LConn,hp.LConn,'autorouting','on');
 wrench=modelName+'/'+name+'_wrench';
 add_block('sm_lib/Forces and Torques/External Force and Torque',wrench, ...
 'EnableForce','on','EnableTorque','on','ForceResolutionFrame','AttachedFrame', ...
 'TorqueResolutionFrame','AttachedFrame','Position',[1920 950+180*i 2020 1020+180*i]);
 wp=get_param(wrench,'PortHandles');add_line(modelName,hp.RConn,wp.RConn,'autorouting','on');
 for kind=1:2
  suffix=["force","torque"];units=["N","N*m"];
  sel=name+'_'+suffix(kind)+'_select';gain=name+'_'+suffix(kind)+'_vector';ps=name+'_'+suffix(kind)+'_ps';
  add_block('simulink/Signal Routing/Selector',modelName+'/'+sel,'NumberOfDimensions','1', ...
   'IndexOptionArray',{'Index vector (dialog)'},'Indices',num2str(i),'InputPortWidth','4', ...
   'Position',[1500 960+180*i+50*(kind-1) 1530 990+180*i+50*(kind-1)]);
  add_block('simulink/Math Operations/Gain',modelName+'/'+gain,'Gain','[0;0;1]','Multiplication','Matrix(K*u)', ...
   'Position',[1570 960+180*i+50*(kind-1) 1620 990+180*i+50*(kind-1)]);
  add_block('nesl_utility/Simulink-PS Converter',modelName+'/'+ps,'Unit',units(kind), ...
   'Position',[1800 960+180*i+50*(kind-1) 1860 990+180*i+50*(kind-1)]);
  wire('sil_actuator_demux',15+kind,sel,1);wire(sel,1,gain,1);wire(gain,1,ps,1);
  pp=get_param(modelName+'/'+ps,'PortHandles');add_line(modelName,pp.RConn,wp.LConn(kind),'autorouting','on');
 end
 color=[0.12 0.32 0.65];if q.reaction_sign(i)>0,color=[0.7 0.18 0.16];end
 addVisual(name+'_motor',q.hubs_m(i,:)+[0 0 -0.010],[0.0179 0.0179 0.0166],[0.18 0.18 0.20],1);
 % Thin ellipsoidal disk represents swept propeller area without added inertia.
 disk=modelName+'/'+name+'_disk';
 add_block('sm_lib/Body Elements/Ellipsoidal Solid',disk,'InertiaType','Custom','Mass','0', ...
  'MomentsOfInertia','[0 0 0]','ProductsOfInertia','[0 0 0]', ...
  'GraphicDiffuseColor',mat2str(color),'GraphicOpacity','0.35', ...
  'Position',[2100 960+180*i 2190 1010+180*i]);
 set_param(disk,'EllipsoidRadii',mat2str([q.diameter_m/2 q.diameter_m/2 0.0006]));
 dp=get_param(disk,'PortHandles');add_line(modelName,hp.RConn,dp.RConn,'autorouting','on');
end
% Legacy visual-only elements remain inertia-free; all mass stays in original links.
save_system(modelName,dest);close_system(modelName,0);
disp('Built '+dest);
 function wire(a,ap,b,bp)
  pa=get_param(modelName+'/'+a,'PortHandles');pb=get_param(modelName+'/'+b,'PortHandles');
  add_line(modelName,pa.Outport(ap),pb.Inport(bp),'autorouting','on');
 end
 function addVisual(name,xyz,dims,color,opacity)
  tr=modelName+'/'+name+'_frame';sol=modelName+'/'+name;
  add_block('sm_lib/Frames and Transforms/Rigid Transform',tr,'TranslationMethod','Cartesian', ...
   'TranslationCartesianOffset',mat2str(xyz,12),'Position',[2300 1000+80*numel(find_system(modelName,'SearchDepth',1,'RegExp','on','Name','quad_leg_.*')) 2360 1050+80*numel(find_system(modelName,'SearchDepth',1,'RegExp','on','Name','quad_leg_.*'))]);
  add_block('sm_lib/Body Elements/Brick Solid',sol,'BrickDimensions',mat2str(dims), ...
   'InertiaType','Custom','Mass','0','MomentsOfInertia','[0 0 0]','ProductsOfInertia','[0 0 0]', ...
   'GraphicDiffuseColor',mat2str(color),'GraphicOpacity',num2str(opacity));
  tp=get_param(modelName+'/torso','PortHandles');rp=get_param(tr,'PortHandles');sp=get_param(sol,'PortHandles');
  add_line(modelName,tp.LConn,rp.LConn,'autorouting','on');add_line(modelName,rp.RConn,sp.RConn,'autorouting','on');
 end
end
