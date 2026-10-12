function [roll,pitch,yaw] = flappy_quaternion_to_euler(quaternion)
%FLAPPY_QUATERNION_TO_EULER Convert scalar-first quaternion rows to ZYX Euler.

quaternion = quaternion./vecnorm(quaternion,2,2);
w = quaternion(:,1); x = quaternion(:,2); y = quaternion(:,3); z = quaternion(:,4);
roll = atan2(2*(w.*x+y.*z),1-2*(x.^2+y.^2));
pitch = asin(min(max(2*(w.*y-z.*x),-1),1));
yaw = atan2(2*(w.*z+x.*y),1-2*(y.^2+z.^2));
end
