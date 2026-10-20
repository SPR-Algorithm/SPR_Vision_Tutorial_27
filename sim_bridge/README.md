# Bevy 仿真、击打闭环与 Foxglove 调试

`src/task1_sim.cpp` 是仿真主流程，运行 `task1_sim`；`task_sim` 保留为同一流程的兼容入口。图像经过 Detector → Solver → Tracker → Planner → Shooter，再由 Talos 把云台角度和单次发射请求送回 Bevy。Foxglove 展示图像、模型和闭环指标。

Target 保持统一的 11 维状态：

```text
[cx, vx, cy, vy, z_ref, vz, theta, omega, r, l, h]
```

前哨站的 `l/h` 是第 1、2 块板相对第 0 块板的高度差；车辆仍为半径差和交替高度差。第 0 块板是首见参考板，不预设高、中、低身份。

## 构建和启动

在本项目根目录运行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

依赖：Eigen >= 3.4、OpenCV、yaml-cpp、fmt、spdlog、nlohmann-json、Boost.System、pthread。OpenVINO 可选；真实 YOLOv5 检测需要它。默认配置保留 `device: GPU`；使用其他设备时自行修改配置。`-DENABLE_OPENVINO=OFF` 时仅支持 `--synthetic`、`--image-only` 和相应 Foxglove 调试，不运行真实图像检测。

在另一终端启动本地模拟器：

```bash
cd /home/firfly/RM/bevy_robomaster_simulator
cargo run --release --no-default-features --features talos --offline -j2 --bin daedalus
```

本机 companion checkout 已包含 Talos 位姿修正、GPU 回读在途限制、mmap 重启处理和发射请求去重扩展。更换 checkout 时需同步 companion 改动；已有的低延迟补丁位于 `sim_bridge/bevy_low_latency.patch`，本机无需重复应用。

回到本项目运行：

```bash
./build/task1_sim --config configs/simulator.yaml
# 兼容入口：./build/task_sim --config configs/simulator.yaml
```

默认只跟踪蓝方前哨，`sim.control: true`、`sim.fire: true`。在 Bevy 按 **F5** 开启自动瞄准订阅后，满足 Shooter 条件才会发射；再次按 F5 停止外部控制和发射。只观察自动瞄准时，把 `sim.fire` 改为 `false`。

`F3` 切换视角；`Shift+C` 依次切换前哨正常旋转、停止和反转。程序没有自动搜索目标功能，需要先把机器人移到无遮挡位置并让相机朝向前哨。

实际场地中，初始区域到蓝方前哨的视线可能被 `TECH_CORE` 和 `POWER` 挡住。当前验证位置的枪口约为 ROS `[2.55, -1.97, 0.29]`，朝向蓝前哨约 yaw `142°` 时，可在 Robot/ThirdPerson 模式按 **A 向左横移约 2～3 m**，释放后等待底盘停止，再重新朝向前哨。参考无遮挡位置 ROS `[0.55, -3.97, 0.29]`，对应 yaw 约 `122°`、包含重力补偿的 pitch 约 `10°`。这些数值只适用于该位置和当前场地，应结合实际枪口位姿调整。Free 模式下 WASD 移动自由相机，不能用它代替底盘移动。

接收线程是 Talos 图像与四路姿态的唯一消费者；不要同时运行第二个真实 IPC 读取程序。图像为 RGB8，接收后转换成具有独立所有权的 BGR8。相机内参和完整世界位姿来自同帧 IPC，覆盖实机标定。当前 Talos v2 图像池固定为 1440×1080。

## 目标角度曲线：Foxglove / PlotJuggler

`task1_sim` 同时发布嵌套的 `target`、`gimbal`、`plan`、`error`，字段含义如下：

| 字段 | 含义 |
| --- | --- |
| `target.yaw/pitch` | 考虑飞行时间和重力的弹道目标角，可与枪口角、规划指令比较 |
| `gimbal.yaw/pitch` | 图像同帧实际枪口世界角度 |
| `plan.yaw/pitch` | MPC 输出角度 |
| `target.phase`、`target.phase_rate` | EKF 的旋转相位 `x[6]` 和角速度 `x[7]`，相位单独画图 |
| `error.yaw/pitch` | 弹道目标角减实际枪口角；yaw 取最短角度差 |

无后缀角度为 rad，`*_deg` 为 degree，pitch 抬头为正。推荐使用 `yaw_unwrapped_deg`，它在 ±180° 处保持连续；`target.phase_unwrapped_deg` 则保留前哨累计旋转。`*.valid` 表示该组数据有效；无效计划省略目标/规划角度，不用零值填曲线。

Foxglove 连接 `ws://127.0.0.1:8765`，新增 **Plot** 面板，将下列三条路径放在同一图中：

```text
/auto_aim/metrics.target.yaw_unwrapped_deg
/auto_aim/metrics.gimbal.yaw_unwrapped_deg
/auto_aim/metrics.plan.yaw_unwrapped_deg
```

Pitch 图改用 `target.pitch_deg`、`gimbal.pitch_deg`、`plan.pitch_deg`；误差图用 `error.yaw_deg`、`error.pitch_deg`。已提供四份面板设置：`foxglove_plots/yaw.json`、`pitch.json`、`error.json`、`phase.json`，在每个 Plot 面板的 **Import/Export Settings** 中导入对应 JSON 即可。面板设置使用接收时间；同一条消息的三条曲线共享同一时间。参见 [Foxglove Plot](https://docs.foxglove.dev/docs/visualization/panels/plot) 和 [面板设置导入说明](https://docs.foxglove.dev/docs/visualization/panels)。

另有官方 SDK 生成的 `foxglove_angles.json`，供支持 programmatic layout v1 的 Notebook/embed 使用；它不是旧版 `configById` 导出格式。`generate_foxglove_angles.py` 可重新生成全部布局/面板设置，仅生成时需要 `foxglove-sdk`，运行 C++ 不需要该依赖。

PlotJuggler 选择 **UDP Server** 数据流，端口 **9870**、解析格式 **JSON**，开始后把 `target/yaw_unwrapped_deg`、`gimbal/yaw_unwrapped_deg`、`plan/yaw_unwrapped_deg` 拖到同一图（插件可能显示额外的路径前缀）。`timestamp` 是采集 UNIX 秒，`time` 是本次运行的单调秒；可按版本支持选择源时间字段。参见 [PlotJuggler 官方项目的数据流与 JSON 支持](https://github.com/PlotJuggler/PlotJuggler)。

```yaml
foxglove:
  rate_hz: 50          # metrics/曲线上限
  visual_rate_hz: 10   # JPEG与3D场景上限
plotjuggler:
  enabled: true
  host: 127.0.0.1
  port: 9870
  rate_hz: 100         # 小型UDP JSON曲线上限
```

曲线和图像分开限频，UDP 不等待查看器，不重试积压包。实际曲线频率受相机/处理帧率限制。`--plotjuggler-port PORT` 可覆盖目的端口，`--no-plotjuggler` 关闭 UDP，`--no-foxglove` 关闭 WebSocket；两个输出可分别使用。

## 可重复的真实前哨测试场景

在已同步 companion 补丁的模拟器目录运行：

```bash
DAEDALUS_TALOS_OUTPOST_TEST=1 cargo run --release --offline --bin daedalus
```

这个可选场景把底盘固定在 ROS `[-2.27, 0.53, 0.29]` 附近，约距蓝前哨 3 m；禁用手动移动/射击并自动开启订阅，因此无需 F5。初始枪口只设置一次，之后由 Talos 指令控制。场景仍使用实际渲染图像与物理弹丸碰撞，目标真值不输入检测、Tracker 或 Planner。停止 C++ 程序或设 `sim.fire: false` 可停止自动发射。未设置该环境变量时使用正常手动/F5场景。

## Planner 与开火控制

Planner 根据实际世界枪口位置计算弹道。预测时间包含图像帧龄、`sim.execution_delay_ms`、规划耗时估计和迭代求解的弹丸飞行时间；在到达时选择可见板，并在一次 MPC 轨迹中保持同一块物理板。轨迹过去部分使用只读解析外推，不调用不支持负时间的 `Target::predict()`。TinyMPC 求解失败、输入非法或弹道不可达时，返回不控制、不发射的计划。

MPC 提供参考轨迹前馈；实际云台反馈由 Shooter 检查。默认配置要求正在跟踪、滤波收敛、当前帧有有效观测、F5 开启、数据新鲜、求解成功、板面可见、实际角度对准，并持续稳定 20 ms；发射间隔至少 80 ms。失锁、临时丢失观测、过期帧或停止跟随会清除就绪状态并禁火。仍有有效预测时可以继续瞄准。

仿真配置使用 `target.measurement_stddev_scale: 2.0` 标定渲染图像/PnP 噪声，协方差按标准差倍率的平方缩放。实机缺省倍率仍为 1，NIS、收敛和开火阈值保持原值；这不是运行时调参器。

本项目 C++ 接口统一使用弧度、**pitch 抬头为正**。Talos 旧协议使用度、pitch 低头为正；`Simulator::send()` 在边界转换符号和单位，Planner/Shooter 不再重复翻转。

Talos 发射扩展复用 v2 `GimbalCmd` 的 padding 前 8 字节，保存 little-endian 的原始发射请求时间戳 token，布局、大小和协议版本不变。后续姿态更新保留最近 token，避免最新帧覆盖擦掉发射脉冲；companion 对 token 去重，原始请求超过 **100 ms** 或来自未来就丢弃，更新姿态时间戳不能延长其有效期。姿态命令本身也有 100 ms 过期检查。token 为零时兼容原有 `fire_advice` 行为；可靠脉冲保留和去重需要使用已同步扩展的 companion。

当前击打验收使用**静止底盘**。Bevy 弹丸会继承底盘速度，Planner 尚未补偿该速度；移动中击打可能产生约 `底盘速度 × 飞行时间` 的落点偏移，不能直接沿用静止底盘的验证结论。

## Foxglove

选择 **Open connection → Foxglove WebSocket**：

```text
ws://127.0.0.1:8765
```

- Image 面板：`/auto_aim/image`。绿色检测框、黄色预测板中心、紫色十字为 Planner 预测击打点。
- 3D 面板：`/auto_aim/scene`，固定坐标系 `world`。橙色为观测，绿色为预测板，蓝色为参考高度处旋转中心。
- Plot / Raw Messages：`/auto_aim/metrics`，按下表排查跟踪、瞄准、发射和时延。

| 指标 | 含义 |
| --- | --- |
| `raw_detected`、`raw_names`、`raw_colors`、`raw_confidences`、`detected` | 网络检测结果与颜色/目标类型筛选后的数量 |
| `tracker_state`、`converged`、`measurement_valid` | 跟踪状态、滤波收敛、当前帧是否有接受的观测 |
| `vyaw`、`radius`、`dh1/dh2`、`z0/z1/z2`、`nis`、`ekf_x`、`P_diag` | 11 维模型与滤波状态 |
| `planned_control`、`solver_ok`、`planner_fire`、`aim_id` | Planner 是否可控制、求解是否成功、开火建议和选板编号 |
| `target_yaw/pitch`、`command_yaw/pitch`、`gun_yaw/pitch` | 弹道目标角、MPC 指令角、实际枪口角，单位 rad |
| `fire_yaw_error_deg`、`fire_pitch_error_deg` | 弹道目标角相对实际枪口的误差，单位 degree |
| `aim_visibility_deg`、`plan_tracking_error`、`fly_time_ms`、`prediction_horizon_ms` | 板面角、MPC 误差、飞行时间和总预测时间 |
| `fire`、`fire_reason`、`fire_stable_ms`、`fire_cooldown_ms` | 本帧发射请求、禁火或发射原因、稳定时间、剩余冷却 |
| `shots_requested` | 本进程发送的发射请求累计数，不保证每个请求都实际出膛 |
| `launch_count`、`hit_count` | Bevy 实际发射和碰撞命中累计数，经 IPC 返回 |
| `frame_age_ms`、`frame_age_p50_ms/p95_ms/p99_ms`、`command_age_ms` | 处理入口帧龄及其分位数、发送前帧龄 |
| `receive_to_process_ms`、`planning_ms`、`processing_ms`、`processing_p95_ms` | 接收到处理、规划和整帧处理耗时 |
| `replaced`、`stale`、`debug_dropped` | 最新帧替换、入口过期帧丢弃、调试数据丢弃 |

常见 `fire_reason`：`not_following` 表示 F5 未开启；`not_tracking`、`not_converged`、`no_measurement` 表示跟踪条件不足；`aim_not_aligned` 表示实际枪口未对准；`planner_hold`、`invalid_plan` 表示规划未允许发射；`settling`、`cooldown` 表示等待稳定或冷却。排查时先确认检测数量和跟踪状态，再看实际枪口误差与发射计数。

`launch_count/hit_count` 是模拟器全局累计计数，包含其他机器人和其他目标，不是前哨专属命中率；两计数也不是原子配对的同帧真值快照。`--synthetic` 只有理想执行器和发射请求模拟，`launch_count` 对应请求数、`hit_count` 始终为 **0**，不代表物理命中验证。

指标默认最多 50 Hz、图像/场景最多 10 Hz 发布，有订阅才编码 JPEG；慢客户端的待发数据被覆盖，不阻塞推理。传输只保留最新完整帧。检测前和控制发送前都检查帧龄，超时不发送控制。分位数覆盖最近最多 1000 个处理帧，每秒更新；`stale` 单独累计入口丢弃的过期帧。

## 独立验证

```bash
# 无需模拟器或神经网络：投影三板 → PnP → Tracker → Planner → Shooter → Foxglove。
# 第 4 秒停止、第 6 秒反转；合成运行不模拟物理命中。
./build/task1_sim --synthetic --duration 9

# 仅检查真实 Bevy IPC、坐标、帧龄和图像，不检测、规划或发射。
./build/task1_sim --image-only --duration 10

# 后台或测试环境不启动调试端口。
./build/task1_sim --synthetic --duration 3 --no-foxglove
```

`--foxglove-port 0` 自动分配端口并打印；`--duration N` 限定运行秒数。Ctrl+C / SIGTERM 退出。

当前时间戳按正常实时运行的 UNIX 时间映射到 C++ 单调时钟；源时钟回退、序号回退或模拟器重启会重置跟踪器。暂停、变速的确定性仿真时钟尚未接入。前哨同帧真值导出仍不完整，场景展示观测与预测，不能据此计算前哨真值误差或专属命中率。

## 验证记录

当前本项目 **9/9 CTest 通过**：Target、Solver、Planner、Shooter、真实 mmap 生产/消费、Foxglove 协议、Plotter UDP、合成主流程和 Foxglove 集成。Planner 测试包含静止、正转、停转、反转、世界平移、实际枪口高度、yaw 环绕和求解失败安全。当前无 OpenVINO 构建的相同 **9/9** 测试也已通过。

Bevy companion 当前 Talos 单测 **8/8**、projectile 单测 **2/2**、Talos IPC 单测 **3/3** 通过。模拟器原有三项模型标签或贴图 legacy 断言仍失败，本次未修改这些模型规则。

固定前哨场景实测：真实 Bevy 渲染 → GPU YOLOv5 → PnP → Tracker → Planner → Shooter → Talos，30 秒处理 1778 帧；开火请求 5 次，模拟器实际发射和装甲碰撞命中计数均由 0 增至 5。入口帧龄 P95 约 33.3 ms、整帧处理 P95 约 20.0 ms。测试场景禁用了手动射击，目标真值未接入算法；这是一次静止底盘场景验证，不构成一般命中率或性能保证。

曲线验证包含 Foxglove 嵌套 schema、单位转换、跨 ±180° 连续展开，以及真实主程序的 PlotJuggler UDP JSON 数据。

本次实景曲线导出到 `records/outpost_angles.csv` 与 `records/outpost_angles.png`（本地验证产物，不入版本管理）；CSV 可在 PlotJuggler 中以 `time` 列为横轴直接加载。
