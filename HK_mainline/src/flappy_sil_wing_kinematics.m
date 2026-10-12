function output = flappy_sil_wing_kinematics(input)
%FLAPPY_SIL_WING_KINEMATICS Analytic prescribed wing motion and derivatives.
% Input: [phase_rad; frequency_hz]. Output: [stroke; pitch; strokeRate;
% pitchRate; strokeAcceleration; pitchAcceleration].  The fixed-frequency
% derivative expressions are exact after the configured 0.5 s spin-up.
% During spin-up they remain bounded and avoid differentiating a sampled
% angle command through an artificial microsecond filter.

arguments
    input (:,1) double {mustBeFinite}
end
assert(numel(input) >= 2,'flappy:wingKinematicsInput', ...
    'Expected phase and frequency.');
phase = input(1);
frequency = max(input(2),0);
omega = 2*pi*frequency;
strokeAmplitude = deg2rad(70);
pitchAmplitude = deg2rad(45);

stroke = strokeAmplitude*sin(phase);
strokeRate = strokeAmplitude*cos(phase)*omega;
strokeAcceleration = -strokeAmplitude*sin(phase)*omega^2;

z = 4*cos(phase);
sechSquared = 1/cosh(z)^2;
pitch = -pitchAmplitude*tanh(z);
pitchRate = 4*pitchAmplitude*sin(phase)*sechSquared*omega;
pitchPhaseAcceleration = 4*pitchAmplitude*sechSquared * ...
    (cos(phase) + 8*sin(phase)^2*tanh(z));
pitchAcceleration = pitchPhaseAcceleration*omega^2;

output = [stroke; pitch; strokeRate; pitchRate; ...
    strokeAcceleration; pitchAcceleration];
end
