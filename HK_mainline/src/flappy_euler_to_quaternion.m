function quaternion = flappy_euler_to_quaternion(euler_rad)
%FLAPPY_EULER_TO_QUATERNION ZYX yaw-pitch-roll, scalar-first quaternion.

euler_rad = euler_rad(:);
roll = euler_rad(1); pitch = euler_rad(2); yaw = euler_rad(3);
cr = cos(roll/2); sr = sin(roll/2);
cp = cos(pitch/2); sp = sin(pitch/2);
cy = cos(yaw/2); sy = sin(yaw/2);
quaternion = [cr*cp*cy + sr*sp*sy; ...
    sr*cp*cy - cr*sp*sy; ...
    cr*sp*cy + sr*cp*sy; ...
    cr*cp*sy - sr*sp*cy];
end
