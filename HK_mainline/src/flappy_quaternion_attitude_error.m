function errorVector = flappy_quaternion_attitude_error(actual,reference)
%FLAPPY_QUATERNION_ATTITUDE_ERROR Small-angle actual-minus-reference error.

actual = actual(:)/norm(actual);
reference = reference(:)/norm(reference);
referenceConjugate = [reference(1);-reference(2:4)];
w1 = referenceConjugate(1); v1 = referenceConjugate(2:4);
w2 = actual(1); v2 = actual(2:4);
errorQuaternion = [w1*w2-dot(v1,v2); w1*v2+w2*v1+cross(v1,v2)];
if errorQuaternion(1) < 0, errorQuaternion = -errorQuaternion; end
errorVector = 2*errorQuaternion(2:4);
end
