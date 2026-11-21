# 仿真与调试组件接口

本目录提供 Bevy/Talos 接收与控制、配置解析及 Foxglove 调试服务。学生在 `src/task1_sim.cpp` 中自行组织调用，不提供已经连接好的自瞄主循环。作业要求与待完成文件见[根 README](../README.md)。

## 启动模拟器

在配套的 `robomaster_simulator` 仓库中运行：

```bash
cargo build --release --no-default-features --features talos --locked -j2 --bin daedalus
cargo run --release --no-default-features --features talos --locked -j2 --bin daedalus
```

首次运行需要下载 Rust 依赖。使用教师配套的 Talos 适配版本；对应补丁保存在 `bevy_low_latency.patch`，已经适配的仓库不需要重复应用。本机配套目录为 `/home/firfly/RM/robomaster_simulator`，其他电脑使用自己的路径。

默认 IPC 文件为 `/tmp/talos_ipc_meta` 和 `/tmp/talos_ipc_image_pool`。适配版本使用 Talos v2，图像池为 RGB8 1440×1080。按 **F5** 切换外部跟随订阅，**F3** 切换视角，**Shift+C** 切换前哨旋转、停止和反向。先让机器人相机朝向无遮挡的目标。自由相机的移动不等于机器人底盘移动。

可选固定前哨场景：

```bash
DAEDALUS_TALOS_OUTPOST_TEST=1 cargo run --release --no-default-features --features talos --locked --bin daedalus
```

该场景固定底盘并自动开启外部订阅，方便重复观察同一目标。它仍输出渲染图像；接口没有提供可用于前哨误差评价的同步目标真值。

## 配置与入口

`load_sim_options(argc, argv)` 声明在 `simulation_config.hpp`，负责命令行和 YAML 解析及参数校验，不负责建立流水线。

```bash
./build/task1_sim --config configs/simulator.yaml
./build/task1_sim --help
```

可用参数包括 `--duration`、`--foxglove-port`、`--plotjuggler-port`、`--no-foxglove`、`--no-plotjuggler`。`task1_sim` 已提供组件初始化，Plotter 和 Foxglove 按配置可选创建；主循环留空，尚不接收数据、发送控制或发布调试内容。`--duration`、帧龄限制和控制开关等循环行为需要学生自行应用。

主要配置分组：`sim` 为 IPC、帧龄和控制参数，`foxglove` 为 WebSocket 地址与发布限频，`plotjuggler` 为 UDP 目的地址与限频；检测与标定配置由对应组件读取。仿真内参和位姿来自同帧 IPC，实机使用现场标定。

## 已有组件

以下列出接口契约，组件连接顺序、线程组织和失效处理由学生设计。

| 组件 | 输入 | 输出或职责 |
| --- | --- | --- |
| `sim_bridge::Simulator` | IPC 目录；读超时；角度命令 | 图像与同帧位姿；向模拟器写入控制请求 |
| `auto_aim::YOLO` | YAML 路径；BGR 图像 | 装甲板类别、颜色、关键点等观测 |
| `auto_aim::Solver` | 标定、相机位姿；装甲板关键点 | 原地填写装甲板空间位置与姿态；`try_solve` 返回是否成功 |
| `auto_aim::Planner` | 公开预测接口、弹速、规划上下文 | 云台角度等规划输出 |
| `auto_aim::Shooter` | 规划、实际云台状态和跟踪有效性 | 发射允许与原因；作业跟随阶段可关闭发射 |
| `sim_bridge::FoxgloveDebug` | 有所有权的图像、JSON、空间点 | 异步发布图像、指标和场景 |
| `tools::Plotter` | JSON | 尽力发送 UDP 曲线数据，不等待查看器 |
| `tools::Exiter`、`tools::logger()` | 退出信号；日志内容 | 退出标记；终端与文件日志 |
| `io::Camera`、`io::Gimbal` | 实机 YAML、硬件 | 相机图像、姿态反馈与云台指令通信；需启用硬件构建 |

完整签名见 `simulator.hpp`、`foxglove_debug.hpp`、`task/auto_aim/*.hpp`、`tools/*.hpp` 和 `io/*.hpp`。通用 EKF 支持自定义函数和差分，学生需要自行定义所用物理模型。

### 算法与规划器的边界

`Target::update(observations, capture_time)` 接收已解算到统一世界坐标系的装甲板观测及采集时刻。`predict(query_time) const` 在指定绝对时刻返回 `Prediction`，查询不能改变滤波器当前状态。预测无效时保持 `valid=false`。

`Prediction::armors` 是候选装甲板列表。每项 `PredictedArmor::xyza` 表示世界位置 x/y/z（m）与装甲板朝向 yaw（rad）；`id` 是学生维护的关联标识，不是模拟器目标真值编号。提供给规划器的候选必须 `valid=true`、`id>=0` 且 `xyza` 全部有限；默认 `id=-1` 不可用于控制。这个输出契约不规定滤波器内部状态维数与排列。

候选朝向沿用 `Solver` 写入的 `Armor::ypr_in_world[0]` 约定。规划器会剔除朝向与枪口到装甲板视线偏差超过 90° 的候选，因此不能把所有装甲板朝向固定为零。

`Tracker::track(observations, capture_time)` 的输入同样已经过空间解算；输出 `optional<Target>`，没有可用目标时返回空。目标管理与失效策略由学生实现。

`Planner::plan(target, bullet_speed, context)` 使用上述只读预测接口，弹速单位为 m/s。`PlanningContext::observation_time` 必须是传给目标更新的采集 `steady_clock` 时刻；`prediction_delay_s` 表示从该时刻到预计命令执行的延迟，不包含弹丸飞行时间；`muzzle_world` 是实际枪口世界位置。飞行时间由规划器计算，不要重复加入。无效规划不会请求控制，学生应检查 `Plan::control`。输出 yaw/pitch 为 rad，pitch 抬头为正。

MPC 会多次查询预测，时刻顺序不保证单调；默认需要覆盖采集时刻到“执行延迟 + 飞行时间 + 约 0.5 s”的范围，不能只支持单个固定预测时长。早于采集时刻的参考采样会夹到采集时刻，不要求学生回推历史；查询不得推进或污染当前滤波状态。

### 图像、空间与时间约定

- `Simulator::read(SimFrame&, timeout)` 成功返回一帧，超时返回 `false`。接收层只保留最新完整帧，跳号不一定代表错误；`stats()` 提供替换、错配、重连等计数。
- 每个 IPC 目录只运行一个 `Simulator` 消费者。不要同时启动两个读取真实 Talos 的程序。
- `SimFrame::bgr` 是独立拥有内存的 BGR8 图像，不是共享内存视图；检测器接收 BGR。
- 世界坐标使用固定 FLU 约定，距离单位为 m；相机光学坐标为右、下、前。`q_world_gimbal` 将云台向量旋转到世界，`q_gimbal_camera` 将相机 link 向量旋转到云台。
- `camera_matrix()`、`distortion_coeffs()` 返回该帧标定；`world_camera_rotation()`、`world_camera_position()` 返回相机 link 的世界位姿。它们可作为 `Solver::set_camera_calibration()` 和 `set_world_camera_pose()` 的输入；后者处理 OpenCV 光学轴转换。
- `Armor::xyz_in_world` 为 m，姿态角为 rad。`Armor::center` 是图像特征，不是可直接当作空间位置使用的几何中心。
- `SimFrame::timestamp` 是采集时间映射到 `steady_clock` 的结果，用于帧间隔和本机延迟计算。`timestamp_ns` 是采集 UNIX 纳秒，供记录与展示；`received_monotonic_ns` 是本机接收时刻。不同时间域不能直接相减。
- `capture_age_ms(frame)` 返回当前帧龄，单位 ms。预测需要区分采集时刻、处理时刻和预计执行时刻。
- `discontinuity` 表示重启或时间/序号不连续；学生应自行设计滤波、跟踪与调试曲线的重置策略。当前时间映射针对正常实时运行，不是暂停或变速的确定性仿真时钟。

### 控制接口

`Simulator::send(yaw_rad, pitch_rad, fire, distance)` 接收世界方向角、发射请求及距离。角度为 rad，pitch 抬头为正，distance 为 m。Talos 协议的角度单位与 pitch 符号转换已经在边界完成，调用者不要重复转换。

`following` 是同帧外部跟随订阅状态。适配模拟器中的旧命令会过期，F5 关闭订阅后停止外部跟随。学生仍需处理失锁、超时和无效输出，不能把过期观测当作新的有效输入。底层 PID 由模拟器提供。跟随实验可将 `sim.fire` 设为 `false`，并在主程序中正确应用该开关。

`launch_count` 和 `hit_count` 只是模拟器全局累计监控计数，包含其他机器人和其他目标；两者不是同步目标真值，也不能直接作为本算法的命中率。实机串口的符号、单位、坐标与模式需和现场电控核对，不能从仿真协议推断。

## Foxglove

创建 `FoxgloveDebug` 并调用 `publish(DebugFrame)` 后，查看器连接：

```text
ws://127.0.0.1:8765
```

默认话题为 `/auto_aim/image`、`/auto_aim/scene`、`/auto_aim/metrics`。Image 面板用于图像，3D 面板固定坐标系选 `world`，Plot 和 Raw Messages 用于学生发布的指标。`DebugFrame` 的 `measured`、`predicted` 是世界坐标空间点；其他可选场景字段仅在自己的算法有相应含义时填写。

`publish()` 接收图像后会在后台编码，发布之后不要再修改该图像像素。发布器只保留最新待处理帧；图像/场景与指标分别限频，有订阅才编码 JPEG。慢查看器不应造成主流程积压。

建议统一曲线命名：

| 字段 | 学生应赋予的含义 |
| --- | --- |
| `target.yaw/pitch` | 目标方向，并在报告注明来自观测还是未来预测 |
| `gimbal.yaw/pitch` | 实际云台或枪口反馈方向，注明使用哪一个 |
| `plan.yaw/pitch` | 规划指令方向 |
| `error.yaw/pitch` | 明确定义的方向误差 |
| `*.valid` | 该组数据本帧是否有效 |

`angle_fields()` 可生成 rad、degree 和连续 yaw 字段；`ContinuousAngle` 可用于跨 ±π 的绘图展开。控制值保持自身约定，曲线展开不应改变控制接口输入。无效数据应省略或标注有效性，不能用零值假装目标回到零位。

采用上述字段后，可在 Plot 面板导入 `foxglove_plots/yaw.json`、`pitch.json`、`error.json`，例如绘制：

```text
/auto_aim/metrics.target.yaw_unwrapped_deg
/auto_aim/metrics.gimbal.yaw_unwrapped_deg
/auto_aim/metrics.plan.yaw_unwrapped_deg
```

这些面板只描述展示路径，不会自动生产数据。学生仍需自行计算并发布字段。

## Plotter 与 PlotJuggler

`tools::Plotter(host, port)` 的默认地址是 `127.0.0.1:9870`，`plot(json)` 发送 JSON。PlotJuggler 中选择 **UDP Server** 数据流、端口 **9870** 和 **JSON** 解析器，再选择学生发布的数值字段绘图。插件显示的路径前缀可能不同。

建议数据中显式带 `timestamp`（采集 UNIX 秒）或 `time`（本次运行的单调秒），并记录具体采用哪个时间轴。学生自行限频，曲线频率受图像和处理帧率限制；UDP 输出不重试积压包。不要把图像或大矩阵塞入高频 UDP 曲线包。

## 自测与实机准备

自行准备静止、横移、停止、反向、前哨旋转和可见板变化等实验，记录输入观测、预测、规划、反馈、有效性、帧龄与耗时。模拟器帮助检查接口与算法行为，最终结果以教师安排的实机验收为准。

本模板没有自动验收测试，也没有完成的参考流水线。硬件驱动可通过 `BUILD_HARDWARE` 构建，但相机采集、时间同步、现场标定和串口控制需在实际设备上核验。

