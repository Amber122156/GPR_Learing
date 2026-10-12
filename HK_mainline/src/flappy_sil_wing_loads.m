function output = flappy_sil_wing_loads(input)
%FLAPPY_SIL_WING_LOADS Generate prescribed test2 motion and world wrenches.
% Input is [phase; frequency; quaternion(wxyz); omega_body(3); velocity_world(3)].
% A legacy two-element input remains valid and evaluates the static-air,
% zero-body-velocity baseline.  The final six outputs record the exact
% Flappy body spatial velocity [p;q;r;u;v;w] used by the aerodynamic kernel.

persistent p
if isempty(p)
    projectRoot = string(fileparts(fileparts(mfilename('fullpath'))));
    p = flappy_parameters(projectRoot);
end
phase = input(1);
frequency = max(input(2),0);
cfg = flappy_sil_active_config();
bodySpatialVelocity = zeros(6,1);
rotationBodyToWorld = eye(3);
if numel(input) >= 12 && cfg.enable_body_velocity_aero
    quaternion = input(3:6);
    omegaBody = input(7:9);
    velocityWorld = input(10:12)-cfg.wind_world_m_s(:);
    rotationBodyToWorld = flappy_quaternion_rotation(quaternion);
    bodySpatialVelocity = [omegaBody(:); rotationBodyToWorld.'*velocityWorld(:)];
elseif numel(input) >= 6
    rotationBodyToWorld = flappy_quaternion_rotation(input(3:6));
end
kinematics = flappy_sil_wing_kinematics([phase; frequency]);
stroke = kinematics(1);
pitch = kinematics(2);
strokeRate = kinematics(3);
pitchRate = kinematics(4);
left = flappy_wing_aero(bodySpatialVelocity,stroke,strokeRate,pitch,pitchRate,"left",p);
right = flappy_wing_aero(bodySpatialVelocity,stroke,strokeRate,pitch,pitchRate,"right",p);
if cfg.disturbance.enabled && cfg.disturbance.enable_wing_mismatch
    left = localScaleWingLoad(left,cfg.disturbance.left_aero_scale);
    right = localScaleWingLoad(right,cfg.disturbance.right_aero_scale);
end
leftTorque = cross(left.application_point_wing_m,left.force_wing_N)+left.pure_moment_wing_Nm;
rightTorque = cross(right.application_point_wing_m,right.force_wing_N)+right.pure_moment_wing_Nm;
% flappy_wing_aero uses the canonical Flappy wing axes.  Imported Simscape
% wing links have a non-identity static visual/link rotation, so resolving
% these vectors in "AttachedFrame" applies a different physical wrench.
% Resolve directly in World instead: the force still acts at each wing root,
% while leftTorque/rightTorque retain the pressure-centre lever arm.
leftCanonical = localRotationZ(stroke)*localRotationY(pitch);
rightCanonical = localRotationZ(-stroke)*localRotationY(pitch);
leftForceWorld = rotationBodyToWorld*leftCanonical*left.force_wing_N;
rightForceWorld = rotationBodyToWorld*rightCanonical*right.force_wing_N;
leftTorqueWorld = rotationBodyToWorld*leftCanonical*leftTorque;
rightTorqueWorld = rotationBodyToWorld*rightCanonical*rightTorque;
output = [stroke; pitch; leftForceWorld; rightForceWorld; leftTorqueWorld; rightTorqueWorld; ...
    bodySpatialVelocity; left.alpha_rad; right.alpha_rad; left.span_cp_m; right.span_cp_m; ...
    left.chord_cp_m; right.chord_cp_m; left.aero_moment_Nm; right.aero_moment_Nm];
end

function rotation = localRotationZ(angle)
rotation = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
end

function rotation = localRotationY(angle)
rotation = [cos(angle) 0 sin(angle); 0 1 0; -sin(angle) 0 cos(angle)];
end

function load = localScaleWingLoad(load,scale)
load.normal_force_N = scale*load.normal_force_N;
load.aero_moment_Nm = scale*load.aero_moment_Nm;
load.rotational_damping_Nm = scale*load.rotational_damping_Nm;
load.force_wing_N = scale*load.force_wing_N;
load.pure_moment_wing_Nm = scale*load.pure_moment_wing_Nm;
end
