function command=flappy_hk_trajectory_command(trajectory,timeS,liftoffPositionM,initialYawRad)
%FLAPPY_HK_TRAJECTORY_COMMAND World-frame reference for paper trajectories.
% The returned yaw is continuous; consumers may wrap it only for display.
base=liftoffPositionM(:);
command=struct('position_world_m',base,'velocity_world_mps',zeros(3,1), ...
    'acceleration_world_mps2',zeros(3,1),'yaw_ref_rad',initialYawRad);
if ~isfield(trajectory,'enabled')||~trajectory.enabled||timeS<trajectory.start_time_s
    if isfield(trajectory,'mode')&&string(trajectory.mode)=="figure8" && ...
            isfield(trajectory,'yaw_preturn_start_s')&&timeS>=trajectory.yaw_preturn_start_s
        u=min(max((timeS-trajectory.yaw_preturn_start_s)/ ...
            (trajectory.start_time_s-trajectory.yaw_preturn_start_s),0),1);
        command.yaw_ref_rad=initialYawRad+trajectory.figure8_initial_yaw_rad*localQuintic(u);
    end
    return
end
elapsed=timeS-trajectory.start_time_s;
executionDuration=localExecutionDuration(trajectory);
if elapsed>executionDuration
    command.yaw_ref_rad=initialYawRad+localFinalYaw(trajectory);
    return
end
elapsed=max(elapsed,0);omega=2*pi/trajectory.period_s;
[theta,thetaRate,thetaAcceleration]=localPhaseSchedule(trajectory,timeS,omega);
mode=string(trajectory.mode);
switch mode
    case "circle"
        command.position_world_m=base+[trajectory.circle_radius_m*sin(theta); ...
            trajectory.circle_radius_m*(1-cos(theta));0];
        command.velocity_world_mps=[trajectory.circle_radius_m*cos(theta)*thetaRate; ...
            trajectory.circle_radius_m*sin(theta)*thetaRate;0];
        command.acceleration_world_mps2=[trajectory.circle_radius_m*(-sin(theta)*thetaRate^2+cos(theta)*thetaAcceleration); ...
            trajectory.circle_radius_m*(cos(theta)*thetaRate^2+sin(theta)*thetaAcceleration);0];
        command.yaw_ref_rad=initialYawRad+theta;
    case "figure8"
        command.position_world_m=base+[trajectory.figure8_x_semi_axis_m*sin(theta); ...
            trajectory.figure8_y_semi_axis_m*sin(2*theta);0];
        command.velocity_world_mps=[trajectory.figure8_x_semi_axis_m*cos(theta)*thetaRate; ...
            2*trajectory.figure8_y_semi_axis_m*cos(2*theta)*thetaRate;0];
        command.acceleration_world_mps2=[trajectory.figure8_x_semi_axis_m*(-sin(theta)*thetaRate^2+cos(theta)*thetaAcceleration); ...
            trajectory.figure8_y_semi_axis_m*(-4*sin(2*theta)*thetaRate^2+2*cos(2*theta)*thetaAcceleration);0];
        command.yaw_ref_rad=initialYawRad+localFigureEightTangentYaw(theta, ...
            trajectory.figure8_x_semi_axis_m*cos(theta), ...
            2*trajectory.figure8_y_semi_axis_m*cos(2*theta));
    otherwise
        % Hover retains the lifted position and initial yaw.
end

function [theta,thetaRate,thetaAcceleration]=localPhaseSchedule(trajectory,timeS,omega)
% A C2 phase schedule starts the vehicle from rest without exceeding the
% eventual cruise angular rate.  The finite entry adds half its duration to
% the manoeuvre time; it does not force a high-acceleration phase catch-up.
entryS=0;
if isfield(trajectory,'smooth_start_s'),entryS=trajectory.smooth_start_s;end
elapsed=max(timeS-trajectory.start_time_s,0);
if entryS<=0 || elapsed>=entryS
    theta=omega*(elapsed-entryS/2);thetaRate=omega;thetaAcceleration=0;
    return
end
u=elapsed/entryS;
% The quintic rate factor is zero at u=0 and one at u=1, with zero
% endpoint acceleration.  Its integral gives the continuous phase.
rateFactor=10*u^3-15*u^4+6*u^5;
phaseIntegral=2.5*u^4-3*u^5+u^6;
rateFactorDerivative=30*u^2-60*u^3+30*u^4;
theta=omega*entryS*phaseIntegral;
thetaRate=omega*rateFactor;
thetaAcceleration=omega*rateFactorDerivative/entryS;
end

function duration=localExecutionDuration(trajectory)
if isfield(trajectory,'execution_duration_s')
    duration=trajectory.execution_duration_s;
    return
end
entryS=0;if isfield(trajectory,'smooth_start_s'),entryS=trajectory.smooth_start_s;end
duration=trajectory.period_s+entryS/2;
end
end

function yaw=localFigureEightTangentYaw(theta,vx,vy)
% atan2 returns an equivalent heading in [-pi,pi], but a figure-eight
% tangent crosses that branch twice.  Select the continuous branch so the
% controller and paper plot do not contain artificial 360-degree steps.
yaw=atan2(vy,vx);
if theta>=3*pi/4 && theta<=5*pi/4
    yaw=yaw-2*pi;
end
end

function value=localQuintic(u)
value=10*u^3-15*u^4+6*u^5;
end

function yaw=localFinalYaw(trajectory)
switch string(trajectory.mode)
    case "circle",yaw=2*pi;
    case "figure8",yaw=trajectory.figure8_initial_yaw_rad;
    otherwise,yaw=0;
end
end
