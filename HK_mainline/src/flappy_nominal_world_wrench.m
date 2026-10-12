function wrench = flappy_nominal_world_wrench(leftLoad,rightLoad,leftStroke,rightStroke,leftPitch,rightPitch,p)
%FLAPPY_NOMINAL_WORLD_WRENCH Resolve prescribed wing loads in the initial body frame.
% The result assumes an identity torso-to-world rotation and is for logging
% only. Simscape applies the actual wing loads in their moving body frames.

leftRotation = rotation_z(leftStroke)*rotation_y(leftPitch);
rightRotation = rotation_z(-rightStroke)*rotation_y(rightPitch);
leftShoulder = [0; p.vehicle.left_shoulder_width; p.vehicle.stroke_plane_offset];
rightShoulder = [0; -p.vehicle.right_shoulder_width; p.vehicle.stroke_plane_offset];
leftForce = leftRotation*leftLoad.force_wing_N;
rightForce = rightRotation*rightLoad.force_wing_N;
leftMoment = leftRotation*(cross(leftLoad.application_point_wing_m, ...
    leftLoad.force_wing_N)+leftLoad.pure_moment_wing_Nm) + cross(leftShoulder,leftForce);
rightMoment = rightRotation*(cross(rightLoad.application_point_wing_m, ...
    rightLoad.force_wing_N)+rightLoad.pure_moment_wing_Nm) + cross(rightShoulder,rightForce);
wrench.force_world_N = leftForce+rightForce;
wrench.moment_world_Nm = leftMoment+rightMoment;
wrench.left_force_body_N = leftForce;
wrench.right_force_body_N = rightForce;
wrench.left_moment_body_Nm = leftMoment;
wrench.right_moment_body_Nm = rightMoment;
end

function rotation = rotation_z(angle)
rotation = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
end

function rotation = rotation_y(angle)
rotation = [cos(angle) 0 sin(angle); 0 1 0; -sin(angle) 0 cos(angle)];
end
