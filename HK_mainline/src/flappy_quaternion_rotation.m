function rotation = flappy_quaternion_rotation(quaternion)
%FLAPPY_QUATERNION_ROTATION Body-to-world rotation for scalar-first quaternions.

quaternion = quaternion(:);
if numel(quaternion) ~= 4 || ~all(isfinite(quaternion)) || norm(quaternion) < 1e-12
    rotation = eye(3);
    return
end
quaternion = quaternion/norm(quaternion);
w = quaternion(1); x = quaternion(2); y = quaternion(3); z = quaternion(4);
rotation = [1-2*(y^2+z^2), 2*(x*y-z*w), 2*(x*z+y*w); ...
    2*(x*y+z*w), 1-2*(x^2+z^2), 2*(y*z-x*w); ...
    2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x^2+y^2)];
end
