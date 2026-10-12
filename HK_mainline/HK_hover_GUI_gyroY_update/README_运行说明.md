# HK GUI 算法开关更新

适用工程：`E:\Matlab_Projections\HK_hover_GUI_20261011\HK_mainline`。

本更新沿用已完成的“同一机体输入”电机响应对比。勾选算法后，先完成原始整机仿真，再将这次记录的相同输入分别送入原控制链和接入算法的控制链。两次回放各自推进控制器、电机及延迟状态；算法处理后的电机输出不反馈到机体动力学。

## 安装与打开

1. 关闭当前 HK GUI 窗口。
2. 将更新包解压到一个独立文件夹，在 MATLAB 中切换到其中包含 `install_hk_gyroY_gui.m` 的目录。
3. 执行：

```matlab
install_hk_gyroY_gui('E:\Matlab_Projections\HK_hover_GUI_20261011\HK_mainline');
app = hk_hover_gui;
```

安装器先把将被替换的文件备份到工程的 `backups/gyroY_gui_时间戳/`，再复制更新文件。无需重建或手工修改 `.slx`。安装后恢复原工作目录为 HK_mainline。

之后每次打开：

```matlab
cd('E:\Matlab_Projections\HK_hover_GUI_20261011\HK_mainline')
app = hk_hover_gui;
```

## 界面操作

“开始仿真”右侧新增“开启算法”复选框，默认关闭。

- 不勾选：运行原仿真，输出原来的高度、水平位置、姿态和四电机转速。
- 勾选后点击“开始仿真”：自动完成原仿真与同输入算法回放，在右侧显示对比。
- “原有输出”页保留四张图；转速图中每台电机同色，虚线为接入前，实线为接入后。
- “逐电机对比”页为四行三列：M1–M4 各一行，油门、转速、推力各一列。蓝色虚线为接入前，橙色实线为接入后。
- “算法信号”页显示原始/分离后的 Gyro Y、估计周期分量、俯仰请求力矩及四旋翼实际俯仰力矩。
- 对比页默认放大末尾 0.5 秒，便于查看扑频振荡；可在“逐电机对比”页切换全程或末尾 2 秒，这也同步改变“算法信号”页的时间范围。
- 指标区保留原飞行指标，并新增 M1–M4 转速谐波频带 RMS 降幅。

高度、水平位置和姿态均标明“基准轨迹”，对应本次原始机体仿真。这里没有第二条接入算法后自由演化的飞行轨迹；电机改善指标不能直接当作整机稳定性或耗能改善。

两个扑翼电机仍没有动力学模型，新增曲线针对现有四个旋翼电机。

## 参数与命令行接口

默认参数与上一版相同：

```matlab
app.AlgorithmConfig.nHarm = 7;
app.AlgorithmConfig.K = 28;
app.AlgorithmConfig.memoryCycles = 8;
app.AlgorithmConfig.freqTau = 0.25;
app.AlgorithmConfig.corrGain = 1;
app.setAlgorithmEnabled(true);
```

其余原算法字段也可按原名修改，例如 `app.AlgorithmConfig.freqBand`。在点击“开始仿真”之前修改，参数在本次启动时确定。GUI 的工况/控制/电机/传感参数编辑及 JSON 读写接口维持原来格式；算法参数单独保存在本次结果目录的 `algorithm_config.json`。

无 GUI 的同一流程：

```matlab
startup_mainline;
p = hk_hover_defaults;
c = gyroY_online_config;
result = run_hover(p,c);     % 整机基准仿真 + 两条同输入回放
% result = run_hover(p);     % 仅原整机仿真
```

原先单独读取 baseline 的接口继续可用：

```matlab
[fn,fp] = uigetfile('*.mat','选择 baseline 的 result.mat');
OUT = run_hk_gyroY_replay(fullfile(fp,fn),gyroY_online_config);
```

## 学习、统计与保存

仍采用原始 500 Hz 控制采样和 2048 点 FFT 窗口，约 4.096 秒后才可能出现首次有效预测。首次有效预测前保留原算法旁路：`clean=raw`，`osc=NaN`。本次不增加滤波、限幅或输出渐入。

算法没有形成有效预测时，界面明确显示旁路，不计算改善指标。稳态评估窗口为 `max(8秒, 首次有效预测后1秒)` 至结束；不足 1 秒时保存曲线，但指标为 NaN。建议沿用已验证的 15 秒工况。工况扑频改变时，频率搜索范围不会自动跟随；需要按原参数接口主动设置。

每次结果仍位于 `results/gui_hover/时间戳/`：

| 内容 | 位置 |
|---|---|
| 原始仿真完整数据与参数 | `result.mat`、`diagnostics.csv`、原 JSON 文件 |
| GUI 原有四张图（含新的转速叠加） | `chart_1.png` 至 `chart_4.png` |
| 本次算法参数 | `algorithm_config.json` |
| 两通道完整回放、逐点与指标数据 | `gyroY_replay/replay.mat` 及 CSV 文件 |
| 逐电机对比图 | `gyroY_replay/motor_compare_01.png` 至 `motor_compare_12.png`，按 M1 油门/转速/推力、M2…排序 |
| 算法信号图 | `gyroY_replay/algorithm_signal_1.png` 至 `algorithm_signal_4.png` |

图像自动保存的是计算结束时所选的时间范围。之后在界面上缩放不会自动覆盖这些图片。

命令行直接查看：

```matlab
R = app.Result.gyroYReplay;
R.baselineCheck
R.metrics
R.algorithmConfig
% R.baseline.actuator 与 R.algorithm.actuator：各为 N×48
% app.Result.diagnostics：原整机基准诊断数据
```

每次算法回放仍先核对基准控制器/电机输出与整机日志；相对误差须小于 1e-7。若不匹配，报告错误并保留已经保存的基准结果，不生成误导性对比。手动停止仿真也只处理已经记录的时段，不外推到原计划时长。回放期间停止按钮关闭，等待计算与保存结束。

## 改动与验证范围

修改 `HKHoverApp.m`、`HKHoverSession.m`、`startup_mainline.m`、`run_hover.m`、`mainline_source_hashes.m`，并附带完整 `HK_gyroY_integration` 文件夹。后者的回放入口新增内存结构体输入、指定输出目录以及短时记录的统计状态；正常 15 秒实验的预测与指标计算公式不变。

`gyroY_online_config.m`、`gyroY_online_init.m`、`gyroY_online_step.m`、`hk_gyroY_sensor_block.m` 和 `verify_gyroY_step.m` 与上一版逐字节一致。控制器、分配器、电机模型及 `.slx` 没有修改。源文件哈希记录新增算法目录，便于追踪每次实验用的代码。

已通过 MATLAB 静态语法检查和上述文件一致性检查。当前交付环境没有 MATLAB/Simulink，无法声称已实测 GUI 显示或完整仿真；安装后首次勾选运行即会执行已有的基准一致性检查。

算法模块文件夹中的 `README_接入说明.md` 保留了算法方程、各通道时序和后续直接接入 Simulink 闭环的说明。直接闭环会改变机体运动，与本 GUI 的同输入回放是不同实验。
