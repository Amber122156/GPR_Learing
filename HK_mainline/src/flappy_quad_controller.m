function out=flappy_quad_controller(input)
% input [sensor27; wingWorldLoad28; previousActuator48]. Output legacy24.
persistent liftTime liftPosition yawRef intV trim eFilter previousVE wingBuffer lastXY aXY lastTime ...
    lastAttitudeOuter arHeld eHeld rateRefHeld
cfg=flappy_sil_active_config(); q=cfg.quad; dt=cfg.control_sample_time_s;
s=input(1:27); wing=input(28:55); act=input(56:103);
t=s(1); pos=s(2:4); vel=s(5:7); quat=s(8:11); rate=s(12:14);
if norm(quat)<1e-9||~all(isfinite(quat)), quat=[1;0;0;0]; end
quat=quat/norm(quat); R=flappy_quaternion_rotation(quat);
if isempty(lastTime)||t<=1e-12
 liftTime=inf; liftPosition=[0;0;cfg.initial_torso_z_m]; yawRef=0;
 intV=0;trim=zeros(3,1);eFilter=trim;previousVE=0;
 wingBuffer=zeros(max(1,round(1/(cfg.takeoff_frequency_hz*dt))),1); lastXY=-inf;aXY=zeros(2,1);lastTime=-inf;
 lastAttitudeOuter=-inf;arHeld=zeros(2,1);eHeld=zeros(3,1);rateRefHeld=zeros(3,1);
end
% The sensor time itself is delayed. Separate clock supplied via sensor has
% 4 ms lag; frequency ramp is computed by actual clock in actuator wrapper.
ramp=min(max(t/cfg.takeoff_ramp_s,0),1); ramp=ramp^2*(3-2*ramp);
% Prescribed wings run at the configured physical flapping frequency.  The
% quadrotor mixer supplies the remaining vertical force; it does not alter
% this command during hover.
freq=cfg.takeoff_frequency_hz*ramp;
wingBuffer=[wingBuffer(2:end);wing(5)+wing(8)];
wingMean=mean(wingBuffer);
if t<0.05, wingMean=q.nominal_wing_lift_N*(freq/cfg.takeoff_frequency_hz)^2*R(3,3); end
if isinf(liftTime)&&t>=cfg.takeoff_ramp_s&&s(15)<cfg.liftoff_normal_threshold_N
 liftTime=t;liftPosition=pos;
 if isfield(cfg,'yaw_reference_mode') && string(cfg.yaw_reference_mode)=="fixed"
     yawRef=cfg.yaw_reference_rad;
 else
     yawRef=atan2(R(2,1),R(1,1));
 end
end
active=isfinite(liftTime);
ref=liftPosition;refVelocity=zeros(3,1);yawCommand=yawRef;
if active
 blend=min(max((t-liftTime)/cfg.reference_ramp_s,0),1); blend=blend^2*(3-2*blend);
 ref(3)=liftPosition(3)+(cfg.hover_height_m-liftPosition(3))*blend;
 if isfield(cfg,'trajectory')&&cfg.trajectory.enabled
     trajectory=flappy_hk_trajectory_command(cfg.trajectory,t,liftPosition,yawRef);
     ref(1:2)=trajectory.position_world_m(1:2);
     refVelocity(1:2)=trajectory.velocity_world_mps(1:2);
     yawCommand=trajectory.yaw_ref_rad;
 else
     ref(1:2)=liftPosition(1:2)+cfg.position_reference_offset_xy_m(:)*blend;
     if t>=cfg.position_step_time_s,ref(1:2)=ref(1:2)+cfg.position_step_delta_xy_m(:);end
 end
end
kp=cfg.position_kp_s2(:);kd=cfg.position_kd_s(:);
pk=cfg.flappy_height_position_to_velocity_kp_s;
vk=cfg.flappy_height_velocity_pid_kp;vi=cfg.flappy_height_velocity_pid_ki;
if cfg.disturbance.enabled
 kp=cfg.disturbance.position_kp_s2(:);kd=cfg.disturbance.position_kd_s(:);
 pk=cfg.disturbance.height_position_to_velocity_kp_s;
 vk=cfg.disturbance.height_velocity_pid_kp;vi=cfg.disturbance.height_velocity_pid_ki;
end
if t-lastXY>=cfg.position_sample_time_s-1e-10
 aXY=-kp.*(pos(1:2)-ref(1:2))-kd.*(vel(1:2)-refVelocity(1:2));
 aXY=min(max(aXY,-cfg.horizontal_acceleration_limit_m_s2),cfg.horizontal_acceleration_limit_m_s2);
 if ~active,aXY(:)=0;end
 lastXY=t;
end
% World acceleration to yaw-aligned body tilt; no zero-yaw assumption.
ah=[cos(yawCommand) sin(yawCommand);-sin(yawCommand) cos(yawCommand)]*aXY;
ar=[atan2(-ah(2),cfg.gravity_mps2);atan2(ah(1),cfg.gravity_mps2)];
% Root-vibration sensitivity cases need a pure attitude command: position
% control remains active for logging but cannot add a hidden pitch/roll bias.
if isfield(cfg,'filter_validation') && isfield(cfg.filter_validation,'freeze_xy_attitude_reference') && ...
        cfg.filter_validation.freeze_xy_attitude_reference
    ar(:)=0;
end
% Validation-only pitch reference injection.  It is guarded by a field that
% exists solely in the independent IMU/notch model, leaving every baseline
% controller run unchanged.
if isfield(cfg,'filter_validation') && cfg.filter_validation.enabled
    ar(2) = ar(2) + localPitchReference(t,cfg.filter_validation);
end
% Independent paper attitude-step case.  This intentionally does not use
% filter_validation: it is a commanded reference experiment, not an IMU or
% notch-filter sensitivity path.  From the requested test start onward it
% owns roll/pitch reference, thereby preventing the XY position loop from
% cancelling the prescribed attitude pulse.
if isfield(cfg,'attitude_test') && cfg.attitude_test.enabled
    [testReference,overrideXY]=localAttitudeTestReference(t,cfg.attitude_test);
    if overrideXY
        ar=testReference;
    end
end
ar=min(max(ar,-cfg.attitude_reference_limit_rad(:)),cfg.attitude_reference_limit_rad(:));
useMultirate=isfield(cfg,'timing')&&isfield(cfg.timing,'enabled')&&cfg.timing.enabled;
rateGain=max(cfg.attitude_kd_Nms_rad(:),1e-8);
if useMultirate
 % PX4-like hierarchy: the outer attitude loop produces a held body-rate
 % reference; the faster inner loop alone requests torque from the mixer.
 if t-lastAttitudeOuter>=cfg.timing.attitude_outer_sample_time_s-1e-12
     arHeld=ar;
     eHeld=flappy_quaternion_attitude_error(quat,flappy_euler_to_quaternion([arHeld;yawCommand]));
     rateRefHeld=-(cfg.attitude_kp_Nm_rad(:)./rateGain).*eHeld;
     lastAttitudeOuter=t;
 end
 ar=arHeld;e=eHeld;
else
 e=flappy_quaternion_attitude_error(quat,flappy_euler_to_quaternion([ar;yawCommand]));
end
eFilter=eFilter+(1-exp(-dt/cfg.attitude_trim_filter_time_s))*(e-eFilter);
zerr=ref(3)-pos(3);
vref=min(max(pk*zerr,-cfg.flappy_height_velocity_reference_limit_m_s),cfg.flappy_height_velocity_reference_limit_m_s);
ve=vref-vel(3);
if active
 % Only allocation residual, not ordinary actuator lag, unwinds integrators.
 aw=act(37:40)-act(41:44);
 intV=intV+dt*(ve+q.antiwindup_gain_s*aw(1)*max(R(3,3),0)/(cfg.mass_kg*max(vi,eps)));
 trimRate=-cfg.attitude_trim_ki_Nm_rad_s(:).*eFilter;
 if isfield(cfg,'attitude_trim_antiwindup_mode') && ...
         string(cfg.attitude_trim_antiwindup_mode)=="conditional"
     % RP-priority allocation can clip yaw on each wing cycle.  Back-
     % calculating that instantaneous residual into yaw trim accumulates a
     % false DC heading bias.  Freeze only an increment that would increase
     % the previous request's infeasibility; increments toward the feasible
     % wrench remain allowed.  Physical allocation and its logs are retained.
     residual=aw(2:4);
     pushesIntoLimit=abs(residual)>1e-5 & residual.*trimRate<0;
     trimRate(pushesIntoLimit)=0;
     trim=trim+dt*trimRate;
 else
     trim=trim+dt*(trimRate+q.antiwindup_gain_s*aw(2:4));
 end
else
 intV=0;trim(:)=0;
end
intV=min(max(intV,-cfg.flappy_height_velocity_pid_integral_limit_m_s),cfg.flappy_height_velocity_pid_integral_limit_m_s);
trim=min(max(trim,-cfg.attitude_trim_limits_Nm(:)),cfg.attitude_trim_limits_Nm(:));
acc=vk*ve+vi*intV+cfg.flappy_height_velocity_pid_kd_s*(ve-previousVE)/dt;
previousVE=ve;
acc=min(max(acc,cfg.flappy_height_acceleration_limit_m_s2(1)),cfg.flappy_height_acceleration_limit_m_s2(2));
if ~active,acc=0.5;end % finite gentle liftoff surplus after spool-up
vertical=cfg.mass_kg*(cfg.gravity_mps2+acc)*ramp;
projection=R(3,3);
if projection<=0.25
    % Sensitivity cases must report a slow/filter-induced loss of attitude
    % authority instead of aborting before their saturation diagnostics are
    % written.  Legacy baseline runs retain the original fail-fast guard.
    if useMultirate
        projection=0.25;
    else
        assert(false,'flappy:quadTilt','Rotor vertical projection below 0.25; cannot meet vertical demand.');
    end
end
collective=max((vertical-wingMean)/projection,0);
if useMultirate
 torque=rateGain.*(rateRefHeld-rate)+trim;
else
 torque=-cfg.attitude_kp_Nm_rad(:).*e-cfg.attitude_kd_Nms_rad(:).*rate+trim;
end
if q.mode=="static",freq=0;collective=0;torque(:)=0;end
out=[freq;torque;ref(3);zerr;e;double(active);aXY;ref(1:2);ar;trim;collective;projection;ve;intV;vertical];
if isfield(cfg,'trajectory')&&cfg.trajectory.enabled
    % Preserve the legacy 24-element controller-to-actuator command and
    % append yaw only as an analysis signal for the trajectory model.
    out=[out;yawCommand];
end
lastTime=t;
end

function pitchRef = localPitchReference(t,v)
% Piecewise-constant, deterministic validation-only reference injection.
if isfield(v,'pitch_reference_schedule')
    schedule=v.pitch_reference_schedule;
    k=find(t>=schedule.time_s,1,'last');
    if isempty(k), pitchRef=schedule.pitch_rad(1); else, pitchRef=schedule.pitch_rad(k); end
elseif t>=v.pitch_step_start_s && t<v.pitch_step_end_s
    pitchRef=v.pitch_step_rad;
else
    pitchRef=0;
end
end

function [reference,overrideXY]=localAttitudeTestReference(t,test)
% Smooth attitude-test pulses for a reproducible in-flight test.
reference=zeros(2,1);
% This is an attitude experiment, not a position-tracking experiment.  Hold
% roll and pitch references at exactly zero from t=0 so the XY outer loop
% cannot introduce a pre-pulse tilt reference during the 0--5 s hover.
overrideXY=true;
if isfield(test,'pitch_pulses')
    % Pitch-only test: roll remains exactly zero after the test begins.
    for k=1:numel(test.pitch_pulses)
        pulse=test.pitch_pulses(k);
        reference(2)=reference(2)+localSmoothPulse(t,pulse.window_s, ...
            pulse.amplitude_rad,test.transition_s);
    end
else
    % Backward-compatible legacy roll/pitch pulse definition.
    reference(2)=localSmoothPulse(t,test.pitch_window_s,test.amplitude_rad,test.transition_s);
    reference(1)=localSmoothPulse(t,test.roll_window_s,test.amplitude_rad,test.transition_s);
end
end

function value=localSmoothPulse(t,window,amplitude,transitionS)
% Raised-cosine edges retain the physical meaning of a step command while
% avoiding an artificial infinite reference-rate impulse at the sample edge.
t0=window(1);t1=window(2);
if t<t0||t>=t1
    value=0;return
end
edge=min([transitionS,(t1-t0)/2]);
if t<t0+edge
    value=amplitude*0.5*(1-cos(pi*(t-t0)/edge));
elseif t>t1-edge
    value=amplitude*0.5*(1-cos(pi*(t1-t)/edge));
else
    value=amplitude;
end
end
