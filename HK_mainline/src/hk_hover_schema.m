function s=hk_hover_schema()
% Single source of units, bounds and defaults for GUI and JSON validation.
% Rows: key, Chinese label (with unit), group, default, minimum, maximum.
r={
'duration_s','仿真时长 (s)','工况',15,.1,120;
'frequency_hz','规定扑翼频率 (Hz)','工况',15,10,40;
'height_m','悬停高度 (m)','工况',.8,.05,5;
'reference_ramp_s','高度参考渐入时间 (s)','工况',1,.05,10;
'takeoff_ramp_s','扑翼起动渐入时间 (s)','工况',.5,.05,10;
'seed','随机种子','工况',20260908,0,4294967295;
'wind_x','世界系风速 X (m/s)','工况',0,-10,10;
'wind_y','世界系风速 Y (m/s)','工况',0,-10,10;
'wind_z','世界系风速 Z (m/s)','工况',0,-10,10;
'z_pos_kp','高度外环 Kp (1/s)','高度控制',1.5,0,20;
'z_vel_kp','垂直速度内环 Kp (1/s)','高度控制',4,0,50;
'z_vel_ki','垂直速度内环 Ki (1/s²)','高度控制',1,0,50;
'z_vel_kd','垂直速度内环 Kd','高度控制',.08,0,5;
'z_integral_limit','速度误差积分上限 (m)','高度控制',1,0,10;
'z_velocity_limit','垂直速度参考上限 (m/s)','高度控制',1,.01,10;
'z_accel_limit','垂直加速度上下限 ± (m/s²)','高度控制',4,.1,20;
'xy_kp_x','X 位置 Kp (1/s²)','水平控制',1.2,0,30;
'xy_kp_y','Y 位置 Kp (1/s²)','水平控制',1.2,0,30;
'xy_kd_x','X 位置 Kd (1/s)','水平控制',1.6,0,30;
'xy_kd_y','Y 位置 Kd (1/s)','水平控制',1.6,0,30;
'xy_accel_limit','水平加速度上限 (m/s²)','水平控制',3,.1,15;
'attitude_limit_deg','横滚/俯仰参考上限 (°)','水平控制',10,1,30;
'roll_kp','横滚 Kp (N·m/rad)','姿态控制',.15,0,5;
'roll_ki','横滚积分修正 Ki (N·m/(rad·s))','姿态控制',.01,0,1;
'roll_kd','横滚 Kd (N·m·s/rad)','姿态控制',.02,0,1;
'pitch_kp','俯仰 Kp (N·m/rad)','姿态控制',.10,0,5;
'pitch_ki','俯仰积分修正 Ki (N·m/(rad·s))','姿态控制',.01,0,1;
'pitch_kd','俯仰 Kd (N·m·s/rad)','姿态控制',.01,0,1;
'yaw_kp','偏航 Kp (N·m/rad)','姿态控制',.02,0,5;
'yaw_ki','偏航积分修正 Ki (N·m/(rad·s))','姿态控制',.002,0,1;
'yaw_kd','偏航 Kd (N·m·s/rad)','姿态控制',.004,0,1;
'trim_filter_s','积分修正低通时间常数 (s)','姿态控制',.05,.002,2;
'wing_enabled','启用左右翼气动力误差','左右翼',false,0,1;
'left_wing_pct','左翼气动力/气动力矩误差 (%)','左右翼',0,-100,100;
'right_wing_pct','右翼气动力/气动力矩误差 (%)','左右翼',0,-100,100;
'motor_delay_ms','电机指令延迟 (ms)','电机',10,0,200;
'motor_tau_ms','电机转速时间常数 (ms)','电机',20,0,500;
'voltage_V','供电电压 (V)','电机',12,1,30;
'motor_kv','电机 KV (rpm/V)','电机',3800,100,10000;
'motor_deadzone','油门死区 (0–1)','电机',.05,0,.5;
'ct_scale','螺旋桨推力系数倍率','电机',1,.1,3;
'cq_scale','螺旋桨反扭矩系数倍率','电机',1,.1,3;
'antiwindup','高度积分回算增益 (1/s)','电机',2,0,30;
'sensor_delay_ms','传感状态延迟 (ms)','传感器',4,2,200;
'noise_enabled','启用传感白噪声','传感器',false,0,1;
'position_noise_m','位置噪声标准差 (m)','传感器',.008,0,.2;
'velocity_noise_mps','速度噪声标准差 (m/s)','传感器',.02,0,1;
'attitude_noise_deg','姿态噪声标准差 (°)','传感器',.25,0,10;
'gyro_noise_deg_s','角速度噪声标准差 (°/s)','传感器',rad2deg(.01),0,100};
s=struct('key',r(:,1),'label',r(:,2),'group',r(:,3),'default',r(:,4),'min',r(:,5),'max',r(:,6));
end
