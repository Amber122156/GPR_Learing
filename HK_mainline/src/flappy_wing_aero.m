function load = flappy_wing_aero(bodySpatialVelocity, strokeAngle, strokeRate, pitchAngle, pitchRate, side, p)
%FLAPPY_WING_AERO Port of Flappy Wing.cpp whole-wing quasi-steady load.
% bodySpatialVelocity is [p;q;r;u;v;w] in upstream body coordinates.

arguments
    bodySpatialVelocity (6,1) double {mustBeFinite}
    strokeAngle (1,1) double {mustBeFinite}
    strokeRate (1,1) double {mustBeFinite}
    pitchAngle (1,1) double {mustBeFinite}
    pitchRate (1,1) double {mustBeFinite}
    side (1,1) string {mustBeMember(side,["left","right"])}
    p (1,1) struct
end

v = p.vehicle;
if side == "left"
    signWing = 1; shoulder = v.left_shoulder_width;
else
    signWing = -1; shoulder = v.right_shoulder_width;
end

roll = bodySpatialVelocity(1); pitch = bodySpatialVelocity(2); yaw = bodySpatialVelocity(3);
u = bodySpatialVelocity(4); lateral = bodySpatialVelocity(5); w = bodySpatialVelocity(6);
psi = strokeAngle; theta = pitchAngle;
sp = 0; spRate = 0; deviation = 0; deviationRate = 0;
Spsi = sin(psi); Cpsi = cos(psi); Sphi = sin(deviation); Cphi = cos(deviation);
SPhi = sin(sp); CPhi = cos(sp);

Uo1 = signWing*(roll*Cpsi*CPhi-spRate*Spsi) + (pitch*Spsi+yaw*Cpsi*SPhi+deviationRate);
Uo0 = signWing*(-(u+pitch*v.stroke_plane_offset)*Cphi*SPhi-yaw*shoulder*Sphi*Spsi*CPhi- ...
    (lateral-roll*v.stroke_plane_offset)*Sphi*Cpsi+w*Sphi*Spsi*SPhi+roll*shoulder*Cphi*CPhi) + ...
    ((u+pitch*v.stroke_plane_offset)*Sphi*Spsi*CPhi+yaw*shoulder*Cphi*SPhi+w*Cphi*CPhi+ ...
    roll*shoulder*Sphi*Spsi*SPhi);
Ui1 = signWing*(roll*Sphi*Spsi*CPhi+yaw*Cphi*CPhi+spRate*Sphi*Cpsi) + ...
    (-roll*Cphi*SPhi-pitch*Sphi*Cpsi+yaw*Sphi*Spsi*SPhi+strokeRate*Cphi);
Ui0 = signWing*(yaw*shoulder*Cpsi*CPhi-(lateral-roll*v.stroke_plane_offset)*Spsi-w*Cpsi*SPhi) + ...
    (-(u+pitch*v.stroke_plane_offset)*Cpsi*CPhi-roll*shoulder*Cpsi*SPhi);

R = v.wing_length; rcp = R*v.r33/v.r22;
Ui = Ui1*rcp+Ui0;
if Ui == 0
    deltaAlpha = 0;
else
    deltaAlpha = atan((Uo1*rcp+Uo0)/(Ui1*rcp+Ui0));
end
alpha = theta + sign(Ui)*pi/2-deltaAlpha;
CN = 1.8*sin(2*alpha)*cos(alpha)+1.95*sin(alpha)-1.5*cos(2*alpha)*sin(alpha);
dcp = 0.46-0.332*cos(alpha)-0.037*cos(3*alpha)-0.013*cos(5*alpha);
aU2 = Ui1^2+Uo1^2; aU1 = 2*Ui1*Ui0+2*Uo1*Uo0; aU0 = Ui0^2+Uo0^2;
rho = v.air_density_kg_m3; chord = v.mean_chord;
normalForce = v.aero_scale*0.5*rho*chord*CN*(aU2*R^3*v.r22+aU1*R^2*v.r11+aU0*R*v.r00);
aeroMoment = -v.aero_scale*0.5*rho*chord^2*CN*dcp*(aU2*R^3*v.z_cp2+aU1*R^2*v.z_cp1+aU0*R*v.z_cp0);
if normalForce == 0, chordCp = 0; else, chordCp = -aeroMoment/normalForce; end
rotationalDamping = -v.aero_scale*0.125*rho*abs(pitchRate)*pitchRate*5*R*chord^4*v.z_rd;

load.alpha_rad = alpha;
load.normal_force_N = normalForce;
load.span_cp_m = rcp;
load.chord_cp_m = chordCp;
load.aero_moment_Nm = aeroMoment;
load.rotational_damping_Nm = rotationalDamping;
load.force_wing_N = [normalForce;0;0];
load.application_point_wing_m = [0;signWing*rcp;-chordCp];
load.pure_moment_wing_Nm = [0;rotationalDamping;0];
end
