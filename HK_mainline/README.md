# HK 扑旋翼悬停参数 GUI

主线已经收敛到 **起飞与定点悬停**。主入口为 `hk_hover_gui.m`，界面中没有圆形、8 字轨迹或俯仰脉冲工况。

模型顺序：**HK CAD / URDF 惯量 → 规定双翼运动 → 机身速度反馈气动力 → Simscape 整机与接地 → 传感状态 → 高度/位置/姿态控制 → 四电机分配、指令延迟与转速滞后 → 曲线与指标**。

## 使用

需要 MATLAB R2024a、Simulink、Simscape、Simscape Multibody。打开解压后的 `HK_mainline` 文件夹，在 MATLAB 命令窗口执行：

```matlab
app = hk_hover_gui;
```

默认 15 Hz、0.80 m、15 s。修改参数后点击 **开始仿真**；运行期间锁定参数，显示仿真时间，结束后更新曲线。**停止并保存已有数据** 可中止长工况并保存已产生的数据。关闭窗口会停止当前模型；要保存中途数据，请先使用停止按钮。

可调参数：高度串级控制（外环 P、垂直速度内环 PID）、X/Y 位置 PD、横滚/俯仰/偏航 PD 与积分修正、左右翼气动力误差、电机指令延迟/转速时间常数/电压/KV/死区/桨系数、传感延迟与白噪声、风速、扑翼频率、悬停高度和渐入时间。

每次运行自动保存至 `results/gui_hover/时间戳/`：输入参数 JSON、生效配置 JSON、结果 MAT、状态 CSV、指标 JSON 和四幅曲线 PNG。参数可用 JSON 保存、读取；修改仅作用于下一次运行。

## 批处理与检查

```matlab
startup_mainline;
validate_mainline;         % GUI 模型质量/惯量/几何 + 推进核心 + 参数/UI 检查
smoke_mainline;            % 1 s 零电机延迟短仿真，不代表悬停稳定性验收
p = hk_hover_defaults();
p.duration_s = 15;
p.wing_enabled = true;
p.left_wing_pct = 3;       % 左翼气动力与气动力矩乘 1.03
p.right_wing_pct = -3;     % 右翼乘 0.97
p.motor_delay_ms = 20;
r = run_hover(p);          % 与 GUI 共用后端和结果格式
```

`run_mainline` 现在仅提供 `gui`（默认）、`hover`、`validate`、`smoke` 四个入口。例如 `run_mainline("hover",Duration=15)`。

## 参数含义与模型范围

左右翼误差是整体气动力/气动力矩比例偏差，不改变翼质量、扑动角度或左右相位。姿态积分项是现有控制器的有界、低通积分修正；水平控制没有人为添加 I 项。电机延迟与传感延迟按 2 ms 控制周期取整，界面显示实际延迟。启用翼误差或噪声后，仍采用界面选定的控制增益。

固定 CAD 总质量为 0.2032 kg（双翼质量/惯量为原始翼参数的 2 倍），扑动 ±70°、翻转接近 ±45°；本版调整规定扑翼频率。气动模型采用准定常刚性翼，四旋翼推进系数仍是待实测标定的先验。电机延迟/时间常数针对辅助四旋翼电机，不是扑翼传动电机。界面显示高度、水平位置、姿态、四电机转速；不额外启动回放或滤波研究工况。

详见 [GUI 操作与结果说明](docs/GUI_GUIDE.md)、[参数与实际代码接口](docs/PARAMETERS.md)、[验证记录](docs/GUI_VALIDATION.md)。

工作目录中保留旧研究源文件和历史入口供追溯；新的 `HK_hover_GUI_20261011.zip` 发布包只携带悬停主线所需模型、源代码和构建依赖，排除历史轨迹/脉冲入口、滤波工况及旧结果。原始 `Flappy_simscape` 目录不受此次修改影响。
