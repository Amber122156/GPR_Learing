function output = flappy_sil_sensor_model(truth)
%FLAPPY_SIL_SENSOR_MODEL Reproducible navigation-state error model.
% Output is [measured 15-state; position bias; velocity bias; attitude bias;
% gyro bias]. The controller consumes the first 15 entries only.

persistent stream positionBias velocityBias attitudeBias gyroBias
cfg = flappy_sil_active_config();
truth = truth(:);
assert(numel(truth) == 15,"flappy:sensorModel","Expected 15 truth-state values.");
t = truth(1);
if isempty(stream) || t <= 1e-12
    stream = RandStream("mt19937ar","Seed",cfg.disturbance.seed);
    positionBias = zeros(3,1); velocityBias = zeros(3,1);
    attitudeBias = zeros(3,1); gyroBias = zeros(3,1);
end

if ~(cfg.disturbance.enabled && cfg.disturbance.enable_sensor_errors)
    output = [truth;zeros(12,1)];
    return
end

dt = cfg.control_sample_time_s;
positionBias = localRandomWalk(positionBias,cfg.disturbance.position_bias_rw_m_sqrt_s, ...
    cfg.disturbance.position_bias_limit_m,dt,stream);
velocityBias = localRandomWalk(velocityBias,cfg.disturbance.velocity_bias_rw_mps_sqrt_s, ...
    cfg.disturbance.velocity_bias_limit_mps,dt,stream);
attitudeBias = localRandomWalk(attitudeBias,cfg.disturbance.attitude_bias_rw_rad_sqrt_s, ...
    cfg.disturbance.attitude_bias_limit_rad,dt,stream);
gyroBias = localRandomWalk(gyroBias,cfg.disturbance.gyro_bias_rw_rad_s_sqrt_s, ...
    cfg.disturbance.gyro_bias_limit_rad_s,dt,stream);

measured = truth;
measured(2:4) = truth(2:4) + positionBias + ...
    cfg.disturbance.position_noise_std_m(:).*randn(stream,3,1);
measured(5:7) = truth(5:7) + velocityBias + ...
    cfg.disturbance.velocity_noise_std_mps(:).*randn(stream,3,1);
attitudeError = attitudeBias + cfg.disturbance.attitude_noise_std_rad(:).*randn(stream,3,1);
measured(8:11) = localQuaternionProduct(truth(8:11),flappy_euler_to_quaternion(attitudeError));
measured(12:14) = truth(12:14) + gyroBias + ...
    cfg.disturbance.gyro_noise_std_rad_s(:).*randn(stream,3,1);
% Contact force is an ideal discrete contact indication in this first-pass
% navigation model; it is not an IMU-derived channel.
output = [measured;positionBias;velocityBias;attitudeBias;gyroBias];
end

function value = localRandomWalk(value,rateStd,limit,dt,stream)
value = value + rateStd(:)*sqrt(dt).*randn(stream,3,1);
value = min(max(value,-limit(:)),limit(:));
end

function quaternion = localQuaternionProduct(a,b)
a = a(:)/norm(a); b = b(:)/norm(b);
quaternion = [a(1)*b(1)-dot(a(2:4),b(2:4)); ...
    a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
quaternion = quaternion/norm(quaternion);
if quaternion(1) < 0, quaternion = -quaternion; end
end
