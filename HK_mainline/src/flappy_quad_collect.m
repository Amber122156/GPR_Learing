function result=flappy_quad_collect(simulation,cfg)
t=(0:cfg.plant_sample_time_s:cfg.duration_s).';
truth=flappy_resample_vector_log(simulation,'sil_truth_state',t,15);
% The vibration diagnostic appends fifteen trace-only values after the
% established 27-state sensor interface.  Keep legacy models fully valid.
try
 sensor=flappy_resample_vector_log(simulation,'sil_sensor_state',t,42);
catch exception
 if ~strcmp(exception.identifier,'flappy:vectorLog'),rethrow(exception);end
 sensor=flappy_resample_vector_log(simulation,'sil_sensor_state',t,27);
end
try
 c=flappy_resample_vector_log(simulation,'sil_controller_output',t,25);
catch exception
 if ~strcmp(exception.identifier,'flappy:vectorLog'),rethrow(exception);end
 c=flappy_resample_vector_log(simulation,'sil_controller_output',t,24);
end
a=flappy_resample_vector_log(simulation,'sil_actuator_output',t,48);
w=flappy_resample_vector_log(simulation,'sil_wing_output',t,28);
[roll,pitch,yaw]=flappy_quaternion_to_euler(truth(:,8:11));
fz=zeros(numel(t),1);
for k=1:numel(t),R=flappy_quaternion_rotation(truth(k,8:11));fz(k)=R(3,3)*a(k,45);end
d=table(t,truth(:,2),truth(:,3),truth(:,4),truth(:,5),truth(:,6),truth(:,7),roll,pitch,yaw, ...
 c(:,5),c(:,13),c(:,14),c(:,10),a(:,1),truth(:,15),w(:,5),w(:,8),fz,w(:,5)+w(:,8)+fz, ...
 'VariableNames',{'time_s','x_m','y_m','z_m','vx_mps','vy_mps','vz_mps','roll_rad','pitch_rad','yaw_rad', ...
 'z_ref_m','x_ref_m','y_ref_m','hover_enabled','frequency_hz','contact_N','wing_left_Fz_world_N','wing_right_Fz_world_N','rotor_Fz_world_N','total_Fz_world_N'});
d.roll_ref_rad=c(:,15);d.pitch_ref_rad=c(:,16);
d.quat_w=truth(:,8);d.quat_x=truth(:,9);d.quat_y=truth(:,10);d.quat_z=truth(:,11);
% Preserve the actual Simscape wing-frame orientations for CAD snapshots.
% Fall back safely for older models that do not log these frame sensors.
try
    qLeft=flappy_resample_vector_log(simulation,'sil_left_wing_quaternion',t,4);
    qRight=flappy_resample_vector_log(simulation,'sil_right_wing_quaternion',t,4);
    d.left_wing_quat_w=qLeft(:,1);d.left_wing_quat_x=qLeft(:,2);d.left_wing_quat_y=qLeft(:,3);d.left_wing_quat_z=qLeft(:,4);
    d.right_wing_quat_w=qRight(:,1);d.right_wing_quat_x=qRight(:,2);d.right_wing_quat_y=qRight(:,3);d.right_wing_quat_z=qRight(:,4);
catch exception
    if ~strcmp(exception.identifier,'flappy:vectorLog'),rethrow(exception);end
end
if size(c,2)>=25,d.yaw_ref_rad=c(:,25);else,d.yaw_ref_rad=zeros(numel(t),1);end
% The optional 25th controller output is trace-only.  Rebuild the yaw
% reference from the same analytic trajectory used by the controller so a
% Simulink diagnostic-port interpolation artefact cannot introduce a false
% +/-360 degree spike in the paper plot or yaw-RMS calculation.
if isfield(cfg,'trajectory')&&cfg.trajectory.enabled
    liftIndex=find(c(:,10)>0.5,1,'first');
    if ~isempty(liftIndex)
        liftPosition=[d.x_m(liftIndex);d.y_m(liftIndex);d.z_m(liftIndex)];
        initialYaw=d.yaw_rad(liftIndex);
        yawReference=zeros(numel(t),1);
        rollTheoryReference=zeros(numel(t),1);
        pitchTheoryReference=zeros(numel(t),1);
        for sample=1:numel(t)
            command=flappy_hk_trajectory_command(cfg.trajectory,t(sample),liftPosition,initialYaw);
            yawReference(sample)=command.yaw_ref_rad;
            % Nominal coordinated-flight attitude: rotate the prescribed
            % world acceleration into the yaw-aligned horizontal frame,
            % then tilt the thrust axis against that acceleration.  This
            % is an analytic trajectory reference, not a noisy PID output.
            aWorld=command.acceleration_world_mps2(1:2);
            yaw=command.yaw_ref_rad;
            aHeading=[cos(yaw),sin(yaw);-sin(yaw),cos(yaw)]*aWorld;
            rollTheoryReference(sample)=atan2(-aHeading(2),cfg.gravity_mps2);
            pitchTheoryReference(sample)=atan2(aHeading(1),cfg.gravity_mps2);
        end
        d.yaw_ref_rad=yawReference;
        d.roll_theory_ref_rad=rollTheoryReference;
        d.pitch_theory_ref_rad=pitchTheoryReference;
    end
end
if ~ismember('roll_theory_ref_rad',d.Properties.VariableNames)
    d.roll_theory_ref_rad=d.roll_ref_rad;
    d.pitch_theory_ref_rad=d.pitch_ref_rad;
end
d.xy_tracking_error_m=sqrt((d.x_m-d.x_ref_m).^2+(d.y_m-d.y_ref_m).^2);
d.yaw_tracking_error_rad=atan2(sin(d.yaw_rad-d.yaw_ref_rad),cos(d.yaw_rad-d.yaw_ref_rad));
for i=1:4
 n="M"+i;
 d.(n+'_rpm')=a(:,12+i)*60/(2*pi); d.(n+'_throttle')=a(:,16+i);
 d.(n+'_delayed_throttle')=a(:,20+i);d.(n+'_thrust_N')=a(:,24+i);
 d.(n+'_reaction_Nm')=a(:,28+i);d.(n+'_equivalent_current_A')=a(:,32+i);
end
for i=1:3
 ax=["x","y","z"];
 d.('tau_'+ax(i)+'_cmd_Nm')=c(:,1+i);d.('tau_'+ax(i)+'_actual_Nm')=a(:,1+i);
 d.('tau_'+ax(i)+'_allocation_residual_Nm')=a(:,37+i)-a(:,41+i);
 d.('trim_'+ax(i)+'_Nm')=c(:,16+i);
end
d.rotor_collective_cmd_N=c(:,20);d.rotor_collective_actual_N=a(:,45);
d.allocation_saturated=a(:,48);
d.lift_estimate_world_N=a(:,46);
d.az_torso_world_mps2=gradient(d.vz_mps,cfg.plant_sample_time_s);
d.truth_p_rad_s=truth(:,12);d.truth_q_rad_s=truth(:,13);d.truth_r_rad_s=truth(:,14);
for n=["wing_left_Fz_world_N","wing_right_Fz_world_N","rotor_Fz_world_N","total_Fz_world_N","az_torso_world_mps2"]
 d.('mean_'+n)=movmean(d.(n),round(0.05/cfg.plant_sample_time_s));
end
ground=flappy_ground_config(flappy_parameters(cfg.project_root));
maxPen=0;
for name=ground.support_names.'
 pen=flappy_resample_vector_log(simulation,'contact_'+name+'_penetration_m',t,1);
 d.('penetration_'+name+'_m')=pen;maxPen=max(maxPen,max(pen));
end
i=find(c(:,10)>0.5,1);life=NaN;if ~isempty(i),life=t(i);end
mask=t>=life+2;valid=any(mask)&&t(end)-(life+2)>=1;
m=struct('liftoff_s',life,'valid_window',valid,'xy_rms_m',NaN,'z_rms_m',NaN, ...
 'roll_rms_deg',NaN,'pitch_rms_deg',NaN,'saturation_fraction',NaN,'max_continuous_saturation_s',NaN, ...
 'finite',all(isfinite(truth),'all'),'max_penetration_m',maxPen,'pass',false);
if valid
 m.xy_rms_m=sqrt(mean((d.x_m(mask)-d.x_ref_m(mask)).^2+(d.y_m(mask)-d.y_ref_m(mask)).^2));
 m.z_rms_m=rms(d.z_m(mask)-d.z_ref_m(mask));m.roll_rms_deg=rad2deg(rms(roll(mask)));m.pitch_rms_deg=rad2deg(rms(pitch(mask)));
 sat=a(mask,48)>0.5;m.saturation_fraction=mean(sat);
 edges=diff([false;sat;false]);m.max_continuous_saturation_s=max([0;(find(edges==-1)-find(edges==1))*cfg.plant_sample_time_s]);
 m.pass=m.finite&&m.xy_rms_m<0.2&&m.z_rms_m<0.2&&m.roll_rms_deg<8&&m.pitch_rms_deg<8&&maxPen<0.005&&m.max_continuous_saturation_s<0.5;
end
if isfield(cfg,'trajectory')&&cfg.trajectory.enabled
 if isfield(cfg.trajectory,'execution_duration_s')
     trajectoryEnd=cfg.trajectory.start_time_s+cfg.trajectory.execution_duration_s;
 else
     smoothStart=0;if isfield(cfg.trajectory,'smooth_start_s'),smoothStart=cfg.trajectory.smooth_start_s;end
     trajectoryEnd=cfg.trajectory.start_time_s+cfg.trajectory.period_s+smoothStart/2;
 end
 trajectoryMask=t>=cfg.trajectory.start_time_s & t<=trajectoryEnd;
 m.trajectory_xy_rms_m=sqrt(mean(d.xy_tracking_error_m(trajectoryMask).^2));
 m.trajectory_xy_max_m=max(d.xy_tracking_error_m(trajectoryMask));
 m.trajectory_yaw_rms_deg=rad2deg(sqrt(mean(d.yaw_tracking_error_rad(trajectoryMask).^2)));
else
 m.trajectory_xy_rms_m=NaN;m.trajectory_xy_max_m=NaN;m.trajectory_yaw_rms_deg=NaN;
end
result=struct('config',cfg,'diagnostics',d,'metrics',m,'sensor',sensor,'controller',c,'actuator',a,'wing',w,'simulation',simulation);
end
