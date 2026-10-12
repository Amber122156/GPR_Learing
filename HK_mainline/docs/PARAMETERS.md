# GUI 参数与仿真接口

界面有 50 个输入。完整可编辑参数以 `hk_hover_schema.m` 为准；输入验证与 GUI 共用该定义。

| 分组 | 字段 | 含义/单位 | 默认值 | 范围 |
|---|---|---|---|---|
| 工况 | `duration_s` | 仿真时长 (s) | `15` | .1 – 120 |
| 工况 | `frequency_hz` | 规定扑翼频率 (Hz) | `15` | 10 – 40 |
| 工况 | `height_m` | 悬停高度 (m) | `.8` | .05 – 5 |
| 工况 | `reference_ramp_s` | 高度参考渐入时间 (s) | `1` | .05 – 10 |
| 工况 | `takeoff_ramp_s` | 扑翼起动渐入时间 (s) | `.5` | .05 – 10 |
| 工况 | `seed` | 随机种子 | `20260908` | 0 – 4294967295 |
| 工况 | `wind_x` | 世界系风速 X (m/s) | `0` | -10 – 10 |
| 工况 | `wind_y` | 世界系风速 Y (m/s) | `0` | -10 – 10 |
| 工况 | `wind_z` | 世界系风速 Z (m/s) | `0` | -10 – 10 |
| 高度控制 | `z_pos_kp` | 高度外环 Kp (1/s) | `1.5` | 0 – 20 |
| 高度控制 | `z_vel_kp` | 垂直速度内环 Kp (1/s) | `4` | 0 – 50 |
| 高度控制 | `z_vel_ki` | 垂直速度内环 Ki (1/s²) | `1` | 0 – 50 |
| 高度控制 | `z_vel_kd` | 垂直速度内环 Kd | `.08` | 0 – 5 |
| 高度控制 | `z_integral_limit` | 速度误差积分上限 (m) | `1` | 0 – 10 |
| 高度控制 | `z_velocity_limit` | 垂直速度参考上限 (m/s) | `1` | .01 – 10 |
| 高度控制 | `z_accel_limit` | 垂直加速度上下限 ± (m/s²) | `4` | .1 – 20 |
| 水平控制 | `xy_kp_x` | X 位置 Kp (1/s²) | `1.2` | 0 – 30 |
| 水平控制 | `xy_kp_y` | Y 位置 Kp (1/s²) | `1.2` | 0 – 30 |
| 水平控制 | `xy_kd_x` | X 位置 Kd (1/s) | `1.6` | 0 – 30 |
| 水平控制 | `xy_kd_y` | Y 位置 Kd (1/s) | `1.6` | 0 – 30 |
| 水平控制 | `xy_accel_limit` | 水平加速度上限 (m/s²) | `3` | .1 – 15 |
| 水平控制 | `attitude_limit_deg` | 横滚/俯仰参考上限 (°) | `10` | 1 – 30 |
| 姿态控制 | `roll_kp` | 横滚 Kp (N·m/rad) | `.15` | 0 – 5 |
| 姿态控制 | `roll_ki` | 横滚积分修正 Ki (N·m/(rad·s)) | `.01` | 0 – 1 |
| 姿态控制 | `roll_kd` | 横滚 Kd (N·m·s/rad) | `.02` | 0 – 1 |
| 姿态控制 | `pitch_kp` | 俯仰 Kp (N·m/rad) | `.10` | 0 – 5 |
| 姿态控制 | `pitch_ki` | 俯仰积分修正 Ki (N·m/(rad·s)) | `.01` | 0 – 1 |
| 姿态控制 | `pitch_kd` | 俯仰 Kd (N·m·s/rad) | `.01` | 0 – 1 |
| 姿态控制 | `yaw_kp` | 偏航 Kp (N·m/rad) | `.02` | 0 – 5 |
| 姿态控制 | `yaw_ki` | 偏航积分修正 Ki (N·m/(rad·s)) | `.002` | 0 – 1 |
| 姿态控制 | `yaw_kd` | 偏航 Kd (N·m·s/rad) | `.004` | 0 – 1 |
| 姿态控制 | `trim_filter_s` | 积分修正低通时间常数 (s) | `.05` | .002 – 2 |
| 左右翼 | `wing_enabled` | 启用左右翼气动力误差 | `false` | 0 – 1 |
| 左右翼 | `left_wing_pct` | 左翼气动力/气动力矩误差 (%) | `0` | -100 – 100 |
| 左右翼 | `right_wing_pct` | 右翼气动力/气动力矩误差 (%) | `0` | -100 – 100 |
| 电机 | `motor_delay_ms` | 电机指令延迟 (ms) | `10` | 0 – 200 |
| 电机 | `motor_tau_ms` | 电机转速时间常数 (ms) | `20` | 0 – 500 |
| 电机 | `voltage_V` | 供电电压 (V) | `12` | 1 – 30 |
| 电机 | `motor_kv` | 电机 KV (rpm/V) | `3800` | 100 – 10000 |
| 电机 | `motor_deadzone` | 油门死区 (0–1) | `.05` | 0 – .5 |
| 电机 | `ct_scale` | 螺旋桨推力系数倍率 | `1` | .1 – 3 |
| 电机 | `cq_scale` | 螺旋桨反扭矩系数倍率 | `1` | .1 – 3 |
| 电机 | `antiwindup` | 高度积分回算增益 (1/s) | `2` | 0 – 30 |
| 传感器 | `sensor_delay_ms` | 传感状态延迟 (ms) | `4` | 0 – 200 |
| 传感器 | `noise_enabled` | 启用传感白噪声 | `false` | 0 – 1 |
| 传感器 | `position_noise_m` | 位置噪声标准差 (m) | `.008` | 0 – .2 |
| 传感器 | `velocity_noise_mps` | 速度噪声标准差 (m/s) | `.02` | 0 – 1 |
| 传感器 | `attitude_noise_deg` | 姿态噪声标准差 (°) | `.25` | 0 – 10 |
| 传感器 | `gyro_noise_deg_s` | 角速度噪声标准差 (°/s) | `rad2deg(.01)` | 0 – 100 |

## 对接关系

- 高度：`flappy_height_position_to_velocity_kp_s` 和 `flappy_height_velocity_pid_*`。
- 水平：`position_kp_s2`、`position_kd_s`。
- 姿态：`attitude_kp_Nm_rad`、`attitude_kd_Nms_rad`、`attitude_trim_ki_Nm_rad_s`。低通积分修正算法和有界力矩沿用现有控制器。
- 翼误差：`disturbance.left_aero_scale` / `right_aero_scale`；仅启用时缩放翼气动力和气动力矩。
- 四旋翼电机：`quad.delay_s`、`quad.time_constant_s`、电气和螺旋桨字段；修改后调用 `flappy_quad_derive`。
- 传感延迟：`sensor_delay_samples` 和真实 Simulink `sil_sensor_delay` 块同时更新；传感延迟至少 2 ms，保持闭环因果性并避免代数环。
- 扰动分支控制增益与主控制增益同步，防止开启翼误差或噪声后发生隐式换参。
- 电机/传感请求延迟均按固定 2 ms 控制周期四舍五入；请求值保存在 `parameters.json`，生效值保存在 `applied_config.json` 和指标文件。

当前不开放 CAD 质量/惯量、扑动/翻转幅值、运动相位、柔性参数、初始倾角等机械输入，避免只有配置改变而保存模型中的物理参数没有同步。质量与幅值在界面和说明中明确为固定值。修改 CAD 后需通过构建流程重新导入。
