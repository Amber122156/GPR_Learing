# HK 仿真器的 Gyro Y 周期分离接入

本包完成两件事：把指定的 `gyroY_online_pipeline_validation.m` 中在线循环改为逐样本函数；使用已保存 baseline 的同一组输入，分别运行原控制器和四旋翼电机模型，比较不接算法/接算法的响应。默认工作流不重新求解机体动力学，不需要重新运行 15 秒 Simscape 仿真。

## 1. 文件放在哪里

把本文件夹完整解压到：

```text
E:\Matlab_Projections\HK_hover_GUI_20261011\HK_mainline\HK_gyroY_integration
```

原来的 HK `src`、`models` 等文件不需要覆盖。将 `20261012_baseline.zip` 解压成一个文件夹，保留其中的 `result.mat`。后面的命令用文件选择窗口定位它，不要求移动 baseline。

## 2. MATLAB 命令

```matlab
cd('E:\Matlab_Projections\HK_hover_GUI_20261011\HK_mainline')
addpath(fullfile(pwd,'HK_gyroY_integration'))

[fn,fp] = uigetfile('*.mat','选择 baseline 中的 result.mat');
baselineFile = fullfile(fp,fn);

c = gyroY_online_config;
c.nHarm = 7;
c.K = 28;
c.memoryCycles = 8;
c.freqTau = 0.25;
c.corrGain = 1;

% 一次性核对：与原脚本在线循环使用完全相同的输入和配置。
check = verify_gyroY_step(baselineFile,c);

% 两次固定输入回放：先原控制链，再接入周期分离。
OUT = run_hk_gyroY_replay(baselineFile,c);
```

`verify_gyroY_step` 对比所有逐样本输出、FFT 接受判定及相位修正记录。它使用原文件中复制的完整在线循环作参照，不把我们后来人工仿真的另一个算法当参照。`pass` 应全部为 1。

`run_hk_gyroY_replay` 首先只回放原控制链，打印它与上传 baseline 的误差。只有相对误差小于 1e-7，才运行算法通道。这是确认对照成立的必要检查；若报基线不匹配，请保留输出，不要放宽阈值。

命令行修改配置可以直接使用原字段名，例如：

```matlab
c.corrGain = 0.8;
OUT = run_hk_gyroY_replay(baselineFile,c);

% 也支持原来的 userCfg 结构体覆盖方式。
OUT = run_hk_gyroY_replay(baselineFile,struct('corrGain',0.8));

% 只保存图片，不弹窗。
c.figureVisible = 'off';
OUT = run_hk_gyroY_replay(baselineFile,c);
```

没有改参数的运行使用以下值：

| 参数 | 值 | 来源 |
|---|---:|---|
| nHarm | 7 | 本次用户指定 |
| K | 28 | 本次用户指定 |
| memoryCycles | 8 | 本次用户指定 |
| freqTau | 0.25 s | 本次用户指定 |
| corrGain | 1 | 本次用户指定 |
| nFFT | 2048 | 原文件 |
| fftHop | 128 | 原文件 |
| freqBand | [8 20] Hz | 原文件 |
| corrLimit | 0.35 rad | 原文件 |
| corrMinCoefficient | 0.5 | 原文件 |
| corrGrid | 128 | 原文件 |

当前 HK 控制/传感采样率为 500 Hz。原 `nFFT=2048` 在这里对应 4.096 s 窗口，hop=128 对应 0.256 s；不会自动改成先前 2000 Hz 下的时间尺度。算法初始化前保留原始旁路行为，此时 clean=raw，osc 为 NaN，valid=0。若 FFT 接受且记忆覆盖充分，才开始在线预测。模块不利用已知 15 Hz 或仿真相位来替代 FFT。

可以以后显式修改 nFFT/fftHop 做另一个实验，但本次默认不修改它们。

## 3. 固定回放如何进入方程

记 baseline 在控制时刻的延迟后传感状态为 s[k]，其中 s(13)=q_raw。两条通道都使用同一个 s[k]、同一条翼载荷历史和同一组模型参数。

```text
baseline:  q_fb[k] = q_raw[k]
algorithm: q_fb[k] = q_raw[k] - d_hat[k]   （valid=1）
           q_fb[k] = q_raw[k]              （原算法尚未有效）
```

当前 HK 姿态控制器的俯仰请求力矩为：

```text
tau_y = -Kp_y * e_R,y - Kd_y * q_fb + trim_y
```

所以在姿态误差、trim 相同的时刻，算法直接造成的差值为：

```text
tau_y,algorithm - tau_y,baseline = +Kd_y * d_hat
```

随后调用完全未修改的 `flappy_quad_allocate` 与 `flappy_quad_actuator`，计算四路油门、延迟油门、转速、推力和反扭矩。不直接修改任何电机输出，也不把 d_hat 当作力矩直接加到电机上。

只替换 q 通道：姿态四元数、位置、速度、p/r、翼载荷均保留 baseline 输入。姿态反馈本身含有的周期分量仍可能驱动电机，因此不能预设电机扑频分量会完全消失。如果发生分配饱和，两个通道内部抗饱和积分状态可以独立分化；不会强制拷贝 baseline 的积分器状态。

时序已按 `.slx` 连接核对：

- `result.sensor` 来自 `sil_sensor_delay` 的输出，已经包含 4 ms 传感延迟，直接使用。
- `result.wing` 来自翼载荷块输出，进入控制器之前再保留原来的 1 个控制周期延迟。
- 控制器的 previousActuator 使用各自通道上一步的执行器输出，不固定为 baseline 输出。
- `result` 数组保存在 0.1 ms 网格，不能把这一网格当算法采样率。程序每隔 20 点读取一次，恢复原始 2 ms 控制时刻；不插值出额外的新 gyro 样本。
- 模块使用实际仿真 Clock。传感向量中的 s(1) 是延迟时间，启动时会重复零，不能拿它作为在线算法的采样时间。

四个旋翼电机输出不反馈到机体状态，因此本次结果是固定输入的执行器对照，不能用来声称飞行轨迹或稳定性已经改善。两个扑翼电机仍不在当前驱动模型内。

## 4. 结果在哪里，怎么看

输出至 HK_mainline 下 `results/gyroY_replay_日期时间/`：

| 文件 | 内容 |
|---|---|
| baseline_check.csv | 固定回放与原 baseline 的一致性 |
| baseline_motor.csv / algorithm_motor.csv | 四个电机的指令、转速、推力、反扭矩等 |
| motor_metrics.csv | 每电机各项均值、去均值 RMS、峰峰值及扑频谐波频带 RMS |
| algorithm_trace.csv | raw、clean、预测周期量、频率、相位、valid、状态、模型版本和训练最大样本索引 |
| fft_log.csv / phase_correction.csv | FFT 判定及相位修正记录 |
| motor_zoom.png | 四行三列：M1–M4 的指令、转速、推力 |
| gyro_and_torque.png | 原始/清理角速度、估计频率、俯仰请求力矩 |
| replay.mat | OUT 结构体，含实际参数、固定输入与两条通道的完整结果 |

指标窗口从 max(8秒,首次有效预测后1秒) 到结束，不包括最后端点。频带为 15 Hz 的 1 到 nHarm 阶谐波，各 ±harmonicBandHalfWidth（默认1 Hz），采用单窗单边周期图计算频带均方值再开方。

`bandReduction_pct = 100*(1 - algorithmBandRMS/baselineBandRMS)`，正值表示该信号频带 RMS 减小，负值表示增加；它不是耗能下降百分比。模型记录的 equivalent_current 仍是负载转速换算的等效电流，不是电调/电池动态电流。请同时看均值和低频变化，避免只追求高频指标变小。

## 5. 将模块直接接入 Simulink（独立的后续闭环实验）

当前先运行第2节的固定回放，不需要改 `.slx`。如果之后要在自由运动的整机闭环中接入，可将 GUI 模型另存为新名字，再做以下连接；这会改变机体运动，不再保证两次实际运动相同。

在 `sil_sensor_delay` 与 `quad_controller_inputs` 第1端口之间插入模块，只改这一条支路，保留原始传感日志支路：

1. 放置 `Clock` 和两输入 `Mux`，按 `[Clock; sil_sensor_delay输出27维]` 排列，得到28维输入。
2. 放置 **Interpreted MATLAB Function** 块，函数为 `hk_gyroY_sensor_block(u)`，Output dimensions 为 `33`，Sample time 为 `0.002`。本实现使用 MATLAB 动态数组与 fminsearch，不使用要求代码生成的 MATLAB Function 块。
3. 输出接 `Demux`，Outputs 设为 `[27 6]`。
4. 前27维接回 `quad_controller_inputs` 第1端口；后6维接 To Workspace，记录 raw、osc、fFiltered、phasePred、valid、appliedCorrection。
5. `quad_controller_inputs` 第2、3端口（翼载荷与上一步执行器反馈）保持原连接。

模块运行前，在 MATLAB 命令窗口设置：

```matlab
global GYROY_CONFIG
GYROY_CONFIG = gyroY_online_config;
GYROY_CONFIG.nHarm = 7;
GYROY_CONFIG.K = 28;
GYROY_CONFIG.memoryCycles = 8;
GYROY_CONFIG.freqTau = 0.25;
GYROY_CONFIG.corrGain = 1;
clear hk_gyroY_sensor_block
```

修改参数后重新开始一次仿真，不在运行中改变 K/H 等状态维度。需由原 HK 启动流程设置 `FLAPPY_SIL_RUNTIME_CONFIG`；仅设置 GYROY_CONFIG 不会配置整机。不要在未接入模块的原 GUI 模型上运行后误以为算法已经启用。若手动另存为新模型运行，可用：

```matlab
global FLAPPY_SIL_RUNTIME_CONFIG
root = pwd;   % 当前目录必须仍为 HK_mainline
p = hk_hover_defaults;
[cfg,info] = hk_hover_config(p,string(root));
FLAPPY_SIL_RUNTIME_CONFIG = cfg;
clear flappy_quad_controller flappy_quad_actuator flappy_sil_sensor_model
clear flappy_sil_wing_loads hk_gyroY_sensor_block
% 新模型的 StopTime、原传感 DelayLength 等保持 baseline 设置。
% sim('你另存并接好线的新模型名');
```

## 6. 改动范围与验证边界

| 文件 | 作用 |
|---|---|
| gyroY_online_config.m | 原参数名字与结构体覆盖接口；仅按用户指示改变默认参数 |
| gyroY_online_init.m | 初始化原在线循环的历史、频率、相位、模板和更新状态 |
| gyroY_online_step.m | 每采样点推进一次原算法；核心局部函数原文复制 |
| hk_gyroY_sensor_block.m | Simulink 的薄接口，仅替换 sensor(13) |
| run_hk_gyroY_replay.m | 恢复 baseline 时序，分别调用原 HK 控制器与执行器，保存对照 |
| verify_gyroY_step.m | 原循环与新逐样本函数的等价性检验 |

源算法 SHA-256：`ca4d1e4d2f9c15a1f3c1c3003887d15f4c22bbe4e061ed6df8546c894a9a946d`。

保留 FFT 峰值判定、频率低通、失效旁路、历史相位回填、固定历史标签、圆周聚类、簇内方差、GPR、互相关方向、修正增益/限幅与每周期更新顺序。核心原有 assert、空簇处理和 Cholesky jitter 原样保留；没有额外加入限幅、滤波、渐入、异常回退或新估计器。逐样本模块只移除了离线绘图、快照和文字事件记录，不参与预测的这些记录由回放脚本按需要重建。

这是桌面 MATLAB/Simulink 模块：为保持原样本索引，状态保存本次运行历史；不是 MCU 定长缓冲或代码生成版本，未模拟算法计算耗时。

交付前已核对核心函数文本一致性并检查 MATLAB 语法。用 Python 独立按现有控制/电机方程核算上传 baseline，控制器最大绝对误差约1.32e-13，执行器48维输出最大绝对误差约3.10e-11，支持时序恢复方式正确。该核算不等于 MATLAB 实际执行：当前环境没有 MATLAB/Simulink，逐样本等价性脚本和接入后的电机改善幅度需要在用户 MATLAB 中运行后确认。
