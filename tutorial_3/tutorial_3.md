# 第三阶段培训：ROS2 工程化与自瞄的数理逻辑

| 项目     | 内容                                                                                            |
| -------- | ----------------------------------------------------------------------------------------------- |
| 教学目标 | 能把识别算法包成 ROS2 节点、与仿真器对接；能从像素四点反推目标位姿，并算出云台能执行的指令      |
| 教学重点 | 节点 / 话题 / 参数 / 功能包与自定义消息 / `cv_bridge`；坐标系转换、PnP、整车建模、EKF、弹道解算 |
| 教学难点 | 「算法与节点分离」的工程习惯；坐标系之间怎么换、解算误差怎么评估                                |
| 考核方式 | 提交第三阶段作业：ROS2 装甲板识别节点（第一部分）+ 位姿解算与弹道解算（第二部分）               |

---

## 目录

- [第三阶段培训：ROS2 工程化与自瞄的数理逻辑](#第三阶段培训ros2-工程化与自瞄的数理逻辑)
  - [配套实例](#配套实例)
  - [总览：自瞄的全部流程](#总览自瞄的全部流程)
  - [一、ROS2 工程化](#一ros2-工程化)
    - [1.1 为什么自瞄要跑在 ROS2 上](#11-为什么自瞄要跑在-ros2-上)
    - [1.2 节点、话题、参数](#12-节点话题参数)
    - [1.3 功能包：`ament_cmake` + `package.xml` + 自定义消息](#13-功能包ament_cmake--packagexml--自定义消息)
    - [1.4 `cv_bridge`：`sensor_msgs::Image` ↔ `cv::Mat`](#14-cv_bridgesensor_msgsimage--cvmat)
    - [1.5 薄节点：把第二阶段的算法包起来](#15-薄节点把第二阶段的算法包起来)
    - [1.6 与 Daedalus 模拟器对接](#16-与-daedalus-模拟器对接)
    - [1.7 常见坑](#17-常见坑)
  - [二、坐标系转换与位姿解算](#二坐标系转换与位姿解算)
    - [2.1 常用坐标系](#21-常用坐标系)
    - [2.2 刚体变换：两个坐标系之间怎么「翻译」](#22-刚体变换两个坐标系之间怎么翻译)
    - [2.3 相机成像模型：从 3D 世界到 2D 图像](#23-相机成像模型从-3d-世界到-2d-图像)
    - [2.4 solvePnP](#24-solvepnp)
    - [2.5 重投影误差与距离估计](#25-重投影误差与距离估计)
  - [三、卡尔曼滤波（Kalman filter）](#三卡尔曼滤波kalman-filter)
    - [3.1 卡尔曼滤波与目标状态估计](#31-卡尔曼滤波与目标状态估计)
    - [3.2 从位姿观测到稳定跟踪](#32-从位姿观测到稳定跟踪)
  - [四、认识并推导 Kalman 滤波器](#四认识并推导-kalman-滤波器)
    - [4.1 直觉：预测与测量相互校正](#41-直觉预测与测量相互校正)
    - [4.2 一维增益：为什么是那个分数](#42-一维增益为什么是那个分数)
    - [4.3 时间下标与模型中的每个量](#43-时间下标与模型中的每个量)
    - [4.4 矩阵 KF 的推导](#44-矩阵-kf-的推导)
    - [4.5 EKF：用函数预测，用雅可比传播误差](#45-ekf用函数预测用雅可比传播误差)
    - [4.6 RoboMaster 四装甲板：从板位姿到整车状态](#46-robomaster-四装甲板从板位姿到整车状态)
      - [4.6.1 明确几何和坐标](#461-明确几何和坐标)
      - [4.6.2 运动模型、观测模型及维度](#462-运动模型观测模型及维度)
      - [4.6.3 可观测性与建模选择](#463-可观测性与建模选择)
    - [4.7 一帧流程速查](#47-一帧流程速查)
  - [五、第三阶段作业](#五第三阶段作业)
  - [六、参考资料](#六参考资料)

---

## 配套实例

本阶段不新增算法示例：识别算法直接复用第二阶段的
[`tutorial_2/example/opencv/armor_detect.cpp`](../tutorial_2/example/opencv/armor_detect.cpp)，
ROS2 工程按下面第一节和作业要求自己搭功能包。

运行环境：Ubuntu 22.04 + ROS2 Humble（安装命令见根目录 [`Readme.md`](../Readme.md) 的「开发环境」一节）。

---

## 总览：自瞄的全部流程

```mermaid
flowchart LR
    subgraph Sensors[传感器与外部状态]
        IC[工业相机]
        UC[USB 辅助相机 × 4]
        IMU[IMU / 云台四元数]
        BS[弹速、模式与弹量]
        NAV[ROS2 导航信息]
    end

    subgraph IO[io 硬件适配层]
        CAM[Camera / USBCamera]
        CB[CBoard / SocketCAN]
        GM[Gimbal / Serial]
        ROS[ROS2 Interface]
    end

    subgraph APP[src 应用层]
        STD[standard]
        MPC[standard_mpc]
        SEN[sentry 系列]
        DBG[debug / test 入口]
    end

    subgraph TASKS[tasks 功能层]
        AA[auto_aim]
        BF[auto_buff]
        OP[omniperception]
    end

    subgraph SUPPORT[通用支撑]
        TL[EKF / 弹道 / 队列 / 录像]
        DB[DebugBus / Web / Plotter]
        CFG[YAML 配置]
    end

    IC --> CAM
    UC --> CAM
    IMU --> CB
    IMU --> GM
    BS --> CB
    BS --> GM
    NAV <--> ROS

    CAM --> APP
    CB <--> APP
    GM <--> APP
    ROS <--> SEN

    APP --> AA
    APP --> BF
    APP --> OP
    AA --> APP
    BF --> APP
    OP --> APP

    TL --> TASKS
    CFG --> IO
    CFG --> APP
    CFG --> TASKS
    APP --> DB
    TASKS --> DB
```

对照这张图看本阶段要学什么：

- **io 层的 `ROS2 Interface`**：本阶段第一节——算法和外界怎么说话。
- **tasks 层的 `auto_aim`**：本阶段第二、三节——四点怎么变成云台角度。
- 第二阶段的成果（`ArmorDetector` 类、`Camera` 抽象类）在这里被当成零件直接调用。

---

## 一、ROS2 工程化

### 1.1 为什么自瞄要跑在 ROS2 上

第二阶段我们已经把「装甲板四点识别」写成了一个类，喂一张图进去、吐四个点出来。
但上车之后它不是孤立的程序，而是整条链路里的一环：

- **上游**：相机（或仿真器）不断给图，还携带相机内参、云台姿态、时间戳；
- **同级**：还有别的节点在跑（能量机关识别、雷达站…）；
- **下游**：弹道解算要拿识别结果，云台驱动要拿解算结果。

这些模块可能由不同人写、用不同语言写、还在不同机器上跑，还要能随时换掉其中一块去调试。
ROS2 提供的就是这套「模块之间怎么说话」的约定：

| 你想要的效果             | ROS2 里对应的东西                  | 自瞄里的例子                                         |
| ------------------------ | ---------------------------------- | ---------------------------------------------------- |
| 上游把图推给我           | **话题 Topic**（发布 / 订阅）      | 订阅 `/image_raw`、发布 `/armor_detector/detections` |
| 我要别人做一件事并等结果 | **服务 Service**                   | 手动触发一次标定 / 重新加载参数                      |
| 启动时改参数、不重编     | **参数 Parameter**                 | 视频路径、二值化阈值、`kSizeWeight`                  |
| 多个模块打包 + 一键启动  | **功能包 Package** + `ros2 launch` | `video_publisher` + `armor_detector` 一起起          |

一句话：**ROS2 不是算法，是工程骨架。** 算法还是第二阶段那一套，只是外面套一层壳。

### 1.2 节点、话题、参数

**节点（node）**就是一个进程，有自己的名字，能收发消息。订阅、发布、参数都在节点上创建：

```cpp
class ArmorDetectorNode : public rclcpp::Node {
public:
  ArmorDetectorNode() : Node("armor_detector") {
    // 1. 参数：能从命令行覆盖，不用改代码重编
    video_path_ = declare_parameter<std::string>("video_path", "demo.mp4");

    // 2. 订阅：注意 QoS 要和发布方对得上
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(msg); });

    // 3. 发布
    pub_det_ = create_publisher<rm_interfaces::msg::ArmorDetections>(
        "/armor_detector/detections", 10);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);          // 一切从 init 开始
  rclcpp::spin(std::make_shared<ArmorDetectorNode>());  // 阻塞，等回调
  rclcpp::shutdown();
  return 0;
}
```

四个必记的调试命令，课上会当场演示：

```bash
ros2 node list                     # 现在有哪些节点
ros2 topic list                    # 现在有哪些话题
ros2 topic hz /image_raw           # 频率对不对（相机是否真的在发）
ros2 topic echo /armor_detector/detections --once   # 消息内容对不对
```

> **QoS 是最容易坑人的地方**：`/image_raw` 是图像流，用的是 `sensor_data`（best effort）。
> 订阅端若用默认的 reliable，就会出现「节点都在跑、就是收不到图」。
> 记住：**视频流类的话题，两端都用 `rclcpp::SensorDataQoS()`。**

### 1.3 功能包：`ament_cmake` + `package.xml` + 自定义消息

一个 ROS2 功能包就是「一个能 `colcon build` 的 CMake 工程」，多两个东西：`package.xml`
（声明依赖）和统一的 `install` 规则。

```bash
ros2 pkg create --build-type ament_cmake armor_detector \
  --dependencies rclcpp sensor_msgs cv_bridge image_transport
```

`CMakeLists.txt` 里关键就这几行（和第一阶段的 CMake 知识是同一套）：

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(armor_detector_node src/armor_detector_node.cpp)
ament_target_dependencies(armor_detector_node rclcpp sensor_msgs cv_bridge)

install(TARGETS armor_detector_node DESTINATION lib/${PROJECT_NAME})
ament_package()
```

**自定义消息（`rosidl`）**：`msg/` 目录下每个 `.msg` 文件就是一个消息类型：

```
# msg/ArmorDetection.msg
Point2d[4] pts
string type
float32 distance_to_image_center
float32 score
```

`.msg` 里能用的基本类型是 `bool int32 float32 string` 等；想用别的包里的类型，
就写完整名字（`std_msgs/Header`、`geometry_msgs/Pose`）。要让别人能用这个包的消息，
`package.xml` 里必须声明：

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

忘了这一条，消息会「生成不出来」，报错却指向另一个文件，属于必踩的坑。

构建与验证：

```bash
colcon build --packages-select armor_detector
source install/setup.bash          # 别忘了！新开终端每次都要 source
ros2 interface show rm_interfaces/msg/ArmorDetections   # 看消息结构对不对
```

### 1.4 `cv_bridge`：`sensor_msgs::Image` ↔ `cv::Mat`

ROS2 的图是 `sensor_msgs::msg::Image`，我们的算法吃的是 `cv::Mat`，中间靠 `cv_bridge` 转：

```cpp
cv_bridge::CvImagePtr cv_ptr;
try {
  // toCvCopy：把数据拷一份出来。后面要二值化、要画线，必须用这个
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
} catch (const cv_bridge::Exception &e) {
  RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败：%s", e.what());
  return;   // 转换失败不能让节点挂掉
}
const cv::Mat &bgr = cv_ptr->image;

// 画完的图发回去
cv_bridge::CvImage out(msg->header, "bgr8", canvas);
pub_dbg_->publish(*out.toImageMsg());
```

> **`toCvCopy` vs `toCvShare`**：`toCvShare` 不拷贝，`cv::Mat` 和消息共用一块内存——
> 你后面`threshold` / `line` 一改，就把消息里的原图改了（还可能把别人共享的数据改坏）。
> 只要会改图，就用 `toCvCopy`。

### 1.5 薄节点：把第二阶段的算法包起来

节点里**不写算法**。回调只做四件事，算法本身一行不改：

```cpp
void onImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
  const cv::Mat bgr = toMat(msg);          // 1. 转换
  const auto armors = detector_->detect(bgr);  // 2. 调用第二阶段写好的算法

  rm_interfaces::msg::ArmorDetections out; // 3. 组装消息
  out.header = msg->header;
  for (const auto &a : armors) { out.detections.push_back(toMsg(a)); }
  pub_det_->publish(out);                  // 4. 发布（空检测也要发）
}
```

为什么一定要这么分？因为这样算法才能**脱离 ROS 单独测**：第二阶段的 `armor_detect`
直接喂图就能跑，将来用 GoogleTest 也能直接测；节点里全是 `rclcpp` 的东西，
一旦掺进算法，就再也离不开 ROS 了。

### 1.6 与 Daedalus 模拟器对接

对接的本质只有两句话：

1. **发布同一个话题**：仿真器发 `/image_raw`（`sensor_msgs/msg/Image`），
   那就把自己的 `video_publisher` 换掉、直接启动仿真器，识别节点一行不用改；
2. **用同一个接口包**：接口用仿真器自带的 `rm_interfaces`，不要另起包名，否则消息类型互认不了。

话题表、`rm_interfaces` 里有什么、验收时怎么切到仿真，见
[`homework_3.md`](homework_3.md) §1.3；仿真器快捷键与结业考核要求见根目录
[`Readme.md`](../Readme.md)。

### 1.7 常见坑

- **QoS 不匹配**：两端不一致就收不到数据（现象是「话题存在但 `hz` 为 0」）。
- **忘了 `source install/setup.bash`**：`ros2 run` 报「找不到包」。
- **图像编码想当然**：`msg->encoding` 可能是 `bgr8` / `rgb8` / `mono8`，写死 BGR8 会颜色错位。
- **在回调里干重活**：`spin` 是单线程的，一帧卡住就掉帧；算法重就上多线程 executor。
- **`package.xml` 漏依赖**：编译过了、运行时找不到库（`ament_target_dependencies` 要写全）。
- **时间戳手填**：用 `this->now()`，别自己造 `builtin_interfaces::msg::Time`。
- **空检测不发消息**：下游会以为你死了。没识别到就发一个空数组。

---

## 二、坐标系转换与位姿解算

**培训目标**：从图像上的四个点，反推出装甲板在空间里的位置与姿态。

### 2.1 常用坐标系

| 坐标系     | 原点固定位置              | x 轴指向  | y 轴指向  | z 轴指向            |
| ---------- | ------------------------- | --------- | --------- | ------------------- |
| 图像坐标系 | 图像左上角                | 向右（u） | 向下（v） | —（2D）             |
| 相机坐标系 | 相机光心                  | 向右（x） | 向下（y） | 向前（光轴方向，z） |
| 云台坐标系 | 云台 yaw/pitch 旋转轴交点 | 向右（x） | 向前（y） | 向上（z）           |
| 世界坐标系 | 云台旋转轴交点            | 向前（x） | 向左（y） | 向上（z）           |

```mermaid
flowchart LR
    UV["图像坐标系<br/>u 向右 · v 向下"]
    CAM["相机坐标系<br/>x 右 · y 下 · z 前"]
    GIM["云台坐标系<br/>x 右 · y 前 · z 上"]
    WLD["世界坐标系<br/>x 前 · y 左 · z 上"]

    UV -->|"相机内参 K"| CAM
    CAM -->|"旋转 R + 平移 t"| GIM
    GIM -->|"旋转 R<br/>原点重合，无平移"| WLD
```

### 2.2 刚体变换：两个坐标系之间怎么「翻译」

$$
P_B = R_{A2B}\, P_A + T_{A2B}
$$

其中：

$$
\begin{aligned}
P_A,\; P_B &\in \mathbb{R}^{3 \times 1} && \text{同一点在 A 系、B 系下的坐标} \\
R_{A2B} &\in \mathbb{R}^{3 \times 3} && \text{旋转矩阵，A 系要「扭」多少才能对上 B 系的方向} \\
T_{A2B} &\in \mathbb{R}^{3 \times 1} && \text{平移向量，A 系原点在 B 系中的位置}
\end{aligned}
$$

> 直觉：先把 A 系中的点按 \(R\) 扭到与 B 系同方向，再把原点偏移 \(T\)。
>
> 旋转矩阵 \(R\) 的一个性质：它是正交矩阵，也就是说 \(R^{-1} = R^{\mathsf{T}}\)。当你需要反向变换（从 B 到 A）时，直接用转置就行：\(P_A = R_{A2B}^{\mathsf{T}}(P_B - T_{A2B})\)。
>
> 而旋转矩阵本身是怎么来的？最常见的方式是用欧拉角（yaw, pitch, roll）来描述。在 RoboMaster 自瞄系统中，这三个角来自云台 IMU，分别表示绕 Z 轴、X 轴、Y 轴的旋转。

### 2.3 相机成像模型：从 3D 世界到 2D 图像

相机本质上是一个 3D → 2D 的投影器。最常用的模型是小孔成像（pinhole model）。

相机坐标系下的点

$$
P_{\mathrm{cam}} = \begin{bmatrix} X \\ Y \\ Z \end{bmatrix}
$$

投影到像素坐标 \((u, v)\)：

$$
u = f_x \frac{X}{Z} + c_x, \qquad
v = f_y \frac{Y}{Z} + c_y
$$

写成内参矩阵：

$$
\begin{bmatrix} u \\ v \\ 1 \end{bmatrix}
=
\frac{1}{Z}\,
K
\begin{bmatrix} X \\ Y \\ Z \end{bmatrix},
\qquad
K =
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

其中：

$$
\begin{aligned}
f_x,\; f_y &\in \mathbb{R} && \text{焦距，以像素为单位} \\
c_x,\; c_y &\in \mathbb{R} && \text{主点，光轴在图像上的落点，通常接近图像中心} \\
\dfrac{X}{Z},\; \dfrac{Y}{Z} &&& \text{归一化坐标：} Z \text{ 越大，点越靠近图像中心（透视效果）}
\end{aligned}
$$

> 这 4 个数 `(fx, fy, cx, cy)` 和径向畸变系数 `(k1, k2, p1, p2, k3)` 共同构成了相机的内参（intrinsic parameters）。标定内参的过程，就是确定这些数的过程。

### 2.4 solvePnP

因为本文的核心动作——从 2D 图像角点反推出 3D 装甲板位置——就是上述过程的反向操作：PnP（Perspective-n-Point）。你把 4 个已知 3D 点的相对位置（装甲板模型点）和它们在图像上的投影（神经网络检出的角点）喂给 `solvePnP`，它就能算出装甲板在相机系下的 3D 位置。

PnP（Perspective-n-Point）是 `solvePnP` 背后的算法。它的输入是：

- 4 个 3D 点（装甲板的物理尺寸——小装甲板 13.5cm × 5.6cm）
- 4 个对应的 2D 投影点（图像中检出的角点）
- 相机内参 \(K\) 和畸变系数

产出是在相机坐标系下的 3D 位置和朝向。

```cpp
cv::solvePnP(centers_3d_, centers_2d, camera_matrix, distort_coeffs,
             rvec, tvec,
             false, cv::SOLVEPNP_IPPE);

// 返回为 rvec tvec
cv::Mat rmat;
// 我们需要将旋转向量和平移向量转化为矩阵形式
cv::Rodrigues(rvec, rmat);
Eigen::Matrix3d R_armor2camera;
// 转化为 eigen 形式矩阵
cv::cv2eigen(rmat, R_armor2camera);
```

> 由此实现了对 3D 信息的获取。
> PnP 算法原理，有兴趣可以看看 https://zhuanlan.zhihu.com/p/698620045。

### 2.5 重投影误差与距离估计

`solvePnP` 给出的 `rvec`、`tvec` 还要过两关：这组解靠不靠谱，以及目标离相机有多远。

**距离。** 平移向量 \(\mathbf{t}\) 就是装甲板模型原点在相机系下的坐标。模型原点放在装甲板中心时，它就是中心点的相机坐标：

$$
\mathbf{t} = \begin{bmatrix} t_x \\ t_y \\ t_z \end{bmatrix},
\qquad
d = \|\mathbf{t}\|_2 = \sqrt{t_x^2 + t_y^2 + t_z^2}
$$

\(d\) 的单位和模型点一致：模型用米，距离就是米。\(t_z\) 只是沿光轴的深度；目标不在画面中心时，斜距 \(d\) 比 \(|t_z|\) 大。后面做弹道要用这个三维点，不能只留一个 \(t_z\)。

**重投影误差。** 用求出的 \(R\)、\(\mathbf{t}\) 把模型点变回相机系，再按 2.3 的内参投回像素，和检出的角点比较：

$$
P_{\mathrm{cam}, i} = R\, P_i + \mathbf{t}
$$

$$
e = \frac{1}{n}\sum_{i=1}^{n}
\left\|
\begin{bmatrix} u_i \\ v_i \end{bmatrix}
-
\begin{bmatrix} u'_i \\ v'_i \end{bmatrix}
\right\|_2
$$

\((u_i, v_i)\) 是检出的角点，\((u'_i, v'_i)\) 是投回去的像素，\(e\) 的单位是像素。角点、模型和内参都对的时候，\(e\) 通常是几个像素；到了十几像素，这一帧不要送进后面的滤波。常见原因是四个角点的顺序和模型点对不上、小装甲板用了大装甲板的尺寸，或内参、畸变不对。

IPPE 对共面的四点会给出两组解，`SOLVEPNP_IPPE` 留下重投影误差更小的那组。误差本身仍要自己算，用来丢掉坏检测。`projectPoints` 会把畸变也算进去，和 `solvePnP` 用的是同一组系数。

```cpp
std::vector<cv::Point2f> projected;
cv::projectPoints(centers_3d_, rvec, tvec, camera_matrix, distort_coeffs, projected);

double sum = 0.0;
for (size_t i = 0; i < centers_2d.size(); ++i) {
  sum += cv::norm(centers_2d[i] - projected[i]);
}
const double reproj_err = sum / static_cast<double>(centers_2d.size()); // 像素
const double distance = cv::norm(tvec); // 与模型点同一单位
```

---

## 三、卡尔曼滤波（Kalman filter）

**为什么需要卡尔曼滤波器？**

在自瞄系统中，你每一帧都能测得目标装甲板的 3D 位置 \((x, y, z)\)。但你有两个问题：

1. 测量值有噪声：神经网络的角点、IMU 的数据都不是完美的，你拿到的 \((x, y, z)\) 会在真实值附近抖动。
2. 你还需要速度：仅仅知道位置不够——你需要知道目标往哪运动、有多快，才能预测它下一步会出现在哪里。

卡尔曼滤波就是同时解决这两个问题的数学工具。它的核心思想只有一句话：

> 「我不完全相信测量值，也不完全相信预测值——我根据各自的噪声大小，在两者之间取一个加权平均。」

你可以用更直觉的方式理解：假设你有两个传感器在测同一个温度，一个精度 ±1°C，另一个精度 ±5°C。当你拿到两个读数时，你会更相信第一个传感器的读数，但也不会完全忽略第二个——卡尔曼滤波做的，就是精确算出这个「加权比例」。

在自瞄系统中，卡尔曼滤波用于从观测到的 3D 位置序列中，估计目标的真实位置和速度。它的输入是 PnP 解算出的 `(x, y, z, yaw)`，输出是平滑后的 `(x, vx, y, vy, z, vz, yaw, vyaw)`——每个维度都包含了位置 + 速度。

### 3.1 卡尔曼滤波与目标状态估计

目的：从逐帧的 4D 观测值 `(x, y, z, yaw)` 中，平滑噪声并估计速度分量 `(vx, vy, vz, vyaw)`。

状态是 8 维，观测只有位置和 yaw，速度由模型推出来：

$$
\mathbf{x} =
\begin{bmatrix}
x & v_x & y & v_y & z & v_z & \mathrm{yaw} & v_{\mathrm{yaw}}
\end{bmatrix}^{\mathsf{T}},
\qquad
\mathbf{z} =
\begin{bmatrix}
x & y & z & \mathrm{yaw}
\end{bmatrix}^{\mathsf{T}}
$$

匀速模型里，每一对「位置–速度」用同一块 \(F_2\) 向前推 \(\Delta t\)，整条状态的转移矩阵由四块拼成：

$$
F_2 = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix},
\qquad
F =
\begin{bmatrix}
F_2 & 0 & 0 & 0 \\
0 & F_2 & 0 & 0 \\
0 & 0 & F_2 & 0 \\
0 & 0 & 0 & F_2
\end{bmatrix}
$$

观测矩阵只挑出 \(x, y, z, \mathrm{yaw}\)，速度所在的列是 0：

$$
H =
\begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 1 & 0
\end{bmatrix},
\qquad
\mathbf{z} = H\mathbf{x}
$$

\(P\) 是状态协方差（\(8\times 8\)）。\(Q\) 表示匀速模型本身有多不准，按加速度白噪声离散化，每个轴一块：

$$
Q_p = \sigma_p^2
\begin{bmatrix}
\Delta t^4 / 4 & \Delta t^3 / 2 \\
\Delta t^3 / 2 & \Delta t^2
\end{bmatrix},
\qquad
Q = \mathrm{blkdiag}(Q_x, Q_y, Q_z, Q_{\mathrm{yaw}})
$$

\(R\) 是 PnP 观测的噪声（\(4\times 4\)）：

$$
R = \mathrm{diag}(\sigma_x^2,\; \sigma_y^2,\; \sigma_z^2,\; \sigma_{\mathrm{yaw}}^2)
$$

**预测**（把上一帧推到这一帧，还没有新观测）：

$$
\hat{\mathbf{x}}_{k|k-1} = F\, \hat{\mathbf{x}}_{k-1|k-1}
$$

$$
P_{k|k-1} = F\, P_{k-1|k-1}\, F^{\mathsf{T}} + Q
$$

**更新**（来了这一帧的 \(\mathbf{z}_k\)，按噪声大小和预测折中）：

$$
\mathbf{y}_k = \mathbf{z}_k - H\, \hat{\mathbf{x}}_{k|k-1}
$$

$$
S_k = H\, P_{k|k-1}\, H^{\mathsf{T}} + R,
\qquad
K_k = P_{k|k-1}\, H^{\mathsf{T}}\, S_k^{-1}
$$

$$
\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + K_k\, \mathbf{y}_k
$$

$$
P_{k|k} = (I - K_k H)\, P_{k|k-1}
$$

\(K_k\) 就是前面说的加权比例。\(R\) 小、观测更准时，\(K_k\) 更大，残差收得更多；预测协方差小、模型更可信时，\(K_k\) 更小，结果更靠近预测。

第一帧没有速度，把 \(\mathbf{z}\) 填进 \(x, y, z, \mathrm{yaw}\)，四个速度置 0。要打提前量时，用预测式再往前推一个飞行时间，这一步不再做更新。

> yaw 的残差要先折进 \((-\pi, \pi]\)。差一个 \(2\pi\) 时，滤波会把目标当成转了一整圈。

### 3.2 从位姿观测到稳定跟踪

3.1 跟的是装甲板中心。车体一转，朝向你的板换成下一块，观测到的 \(x\)、\(y\)、\(\mathrm{yaw}\) 会跳变，速度被拉飞。要跟的是车体中心：它接近匀速，装甲板只是中心外面、半径 \(r\) 处的一个点。观测和状态不再是同一组量，观测函数变成非线性的，滤波从卡尔曼换成扩展卡尔曼（EKF）。

**状态与运动模型。** 11 维。前 8 维是中心的位置、速度和当前这块板的朝向；后 3 维是几何量，两对装甲板可以不一样：

$$
\mathbf{x} =
\begin{bmatrix}
x & v_x & y & v_y & z & v_z & a & w & r & l & h
\end{bmatrix}^{\mathsf{T}}
$$

| 符号     | 含义                                               |
| -------- | -------------------------------------------------- |
| \(x, y\) | 旋转中心的水平位置                                 |
| \(z\)    | 当前这对装甲板的高度                               |
| \(a\)    | 当前这块板的 yaw，从世界 \(x\) 轴转向 \(y\) 轴为正 |
| \(w\)    | yaw 角速度                                         |
| \(r\)    | 当前这对板（\(a\) 与 \(a+\pi\)）到中心的水平半径   |
| \(l\)    | 另一对板（\(a\pm\pi/2\)）到中心的水平半径          |
| \(h\)    | 另一对板相对当前高度的差，另一对的高度是 \(z+h\)   |

中心匀速，\(a\) 按匀角速度转，\(r, l, h\) 视为不变：

$$
F = \mathrm{blkdiag}(F_2, F_2, F_2, F_2, 1, 1, 1),
\qquad
F_2 = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}
$$

预测式和 3.1 相同，\(\hat{\mathbf{x}}_{k|k-1} = F\hat{\mathbf{x}}\)，\(P_{k|k-1} = FPF^{\mathsf{T}} + Q\)。

**观测模型。** PnP 给的是当前这块装甲板。法向取 \((\cos a,\; \sin a)\)，板心在中心沿法向外偏 \(r\)。观测函数记作 \(h(\mathbf{x})\)，和状态里的高度差 \(h\) 不是同一个量：

$$
h(\mathbf{x}) =
\begin{bmatrix}
x + r\cos a \\
y + r\sin a \\
z \\
a
\end{bmatrix}
$$

当前这块板只用 \(r\) 和 \(z\)。\(l\) 和高度差 \(h\) 要等跳到另一对板、把它们换进 \(r\) 和 \(z\) 之后才会被更新。雅可比 \(H = \partial h / \partial \mathbf{x}\) 是 \(4\times 11\)，列顺序与状态相同，\(l\) 和高度差这两列是 0：

$$
H =
\begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 & -r\sin a & 0 & \cos a & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 & r\cos a & 0 & \sin a & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

更新时残差用非线性的 \(h\)，增益用这个 \(H\)（在 \(\hat{\mathbf{x}}_{k|k-1}\) 处取值）：

$$
\mathbf{y}_k = \mathbf{z}_k - h(\hat{\mathbf{x}}_{k|k-1})
$$

$$
S_k = H P_{k|k-1} H^{\mathsf{T}} + R,
\qquad
K_k = P_{k|k-1} H^{\mathsf{T}} S_k^{-1}
$$

$$
\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + K_k \mathbf{y}_k,
\qquad
P_{k|k} = (I - K_k H)\, P_{k|k-1}
$$

\(a\) 那一维的残差仍然先折进 \((-\pi, \pi]\)。

四块板先在状态的一份拷贝上试跳板 \(\delta\)，再拿跳完之后的状态算 \(h(\mathbf{x})\) 和 \(H\)。选中之后才写回状态，中心的 \(x, y\) 不跳。

| \(\delta\)   | 写回前怎么改                                                                                          |
| ------------ | ----------------------------------------------------------------------------------------------------- |
| \(0\)        | 仍是当前这块板                                                                                        |
| \(\pi\)      | \(a \leftarrow a+\pi\)。对面板，同一对，\(r, l, z, h\) 都不动                                         |
| \(\pm\pi/2\) | 换到另一对：\(a \leftarrow a+\delta\)，\(z \leftarrow z+h\)，\(h \leftarrow -h\)，\(r\) 与 \(l\) 互换 |

**马氏距离匹配。** 一帧里可能有多块板、多个车，还要在跳板假设里挑一个。用残差和创新协方差 \(S_k\) 做马氏距离：

$$
d_M^2 = \mathbf{y}_k^{\mathsf{T}} S_k^{-1} \mathbf{y}_k
$$

\(S_k\) 大（刚初始化、跟丢了一阵，或目标很远、观测很吵）时，同样的米制误差 \(d_M^2\) 更小，门更宽；锁紧之后门自动变窄。4 维观测、95% 的卡方门限是 \(9.49\)。对每个检测、每个 \(\delta\) 算一次 \(d_M^2\)，取得过门限的最小值。一个都没过，这一帧只预测、不更新；连续多帧不过就删掉这条航迹。

**噪声。** \(Q\) 仍按 3.1 的加速度白噪声来离散化，按通道分开给：

| 通道              | 怎么给                                                     |
| ----------------- | ---------------------------------------------------------- |
| \(v_x, v_y, v_z\) | 地面车加速度不大，\(\sigma\) 给小                          |
| \(w\)             | 小陀螺会突然起转，\(\sigma_w\) 要比平移大，否则 yaw 跟不住 |
| \(r, l, h\)       | 几何量几乎不变，过程噪声给得很小，只让它们慢慢收敛         |

\(R\) 随距离变。像素误差 \(\sigma_{\mathrm{pix}}\) 投到物方：横向大约正比于深度，沿光轴的深度误差还多一个「板宽 \(L\)」：

$$
\sigma_{xy} \approx \frac{z}{f_x}\sigma_{\mathrm{pix}},
\qquad
\sigma_z \approx \frac{z^2}{f_x L}\sigma_{\mathrm{pix}}
$$

\(f_x\) 用像素，\(z\) 和 \(L\) 用同一长度单位。近处 \(R\) 小、更信观测；远处 \(R\) 大、更信预测。马氏距离用的就是这个 \(R\) 加进 \(S_k\) 之后的门，所以远距离不会因为固定的米制阈值被误删。

\(r\) 和 \(l\) 的初值都用车体半径的大致尺寸（零点几米），高度差 \(h\) 初值取 0。第一帧把装甲板位置减去 \(r(\cos a,\; \sin a)\) 得到中心，速度和 \(w\) 置 0。

---

## 四、认识并推导 Kalman 滤波器

从“一次测量该信多少”出发，逐步走到矩阵 KF、EKF，以及 RoboMaster 四装甲板整车建模。文中向量默认是列向量，$T$ 表示转置，协方差均针对估计误差定义。

### 4.1 直觉：预测与测量相互校正

机器人有两条关于当前位置的信息：上一帧状态经运动模型得到的**预测**，以及传感器给出的**测量**。两者都带误差。Kalman 滤波器根据各自的不确定性计算权重：预测越准，越保留预测；测量越准，越靠近测量。随后更新不确定性，再进入下一帧。

在线性、高斯、模型正确的条件下，KF 给出最小均方误差估计；在更一般的条件下，它仍可理解为一种基于协方差的递推融合方法。EKF 则在当前估计附近对非线性模型作一阶近似。

### 4.2 一维增益：为什么是那个分数

设预测为 $\hat x^-$、预测误差方差为 $P^-$；直接测到 $z=x+v$，其中 $E[v]=0$、$\operatorname{Var}(v)=R$。预测误差与测量噪声互不相关。取加权更新

$$
\hat x^+=\hat x^-+K(z-\hat x^-)=(1-K)\hat x^-+Kz.
$$

令预测误差 $e^-=x-\hat x^-$，则更新误差 $e^+=(1-K)e^--Kv$，因此

$$
P^+(K)=(1-K)^2P^-+K^2R.
$$

对 $K$ 求导并令其为零：

$$
\frac{dP^+}{dK}=-2(1-K)P^-+2KR=0
\quad\Longrightarrow\quad
\boxed{K=\frac{P^-}{P^-+R}}.
$$

代回得到 $P^+=P^-R/(P^-+R)=(1-K)P^-$。例如 $\hat x^-=10$、$P^-=4$、$z=13$、$R=9$，则 $K=4/13$，$\hat x^+=10+12/13\approx10.923$，$P^+=36/13\approx2.769$。方差 $4$ 与 $9$ 不是标准差；这里测量较不精确，所以修正幅度有限。

### 4.3 时间下标与模型中的每个量

$\hat x_{a|b}$ 表示“估计 **$a$ 时刻**状态，已使用截至 **$b$ 时刻**的数据”。因此 $k-1|k-1$ 是上一帧更新后，$k|k-1$ 是本帧预测后、尚未使用本帧测量，$k|k$ 是本帧更新后。$P$ 使用相同下标；上节的上标 $-$、$+$ 分别是 $k|k-1$、$k|k$ 的简写。

设状态 $x_k\in\mathbb R^n$、观测 $z_k\in\mathbb R^m$，线性模型为

$$
x_k=F_kx_{k-1}+w_k,\qquad z_k=H_kx_k+v_k.
$$

| 量             | 维度                   | 含义                                           |
| -------------- | ---------------------- | ---------------------------------------------- |
| $F_k$          | $n\times n$            | 状态转移矩阵：上一帧状态如何变为本帧状态       |
| $H_k$          | $m\times n$            | 线性观测矩阵：状态如何生成观测                 |
| $P_{a\vert b}$ | $n\times n$            | 状态估计误差协方差，既含各量方差也含相互相关性 |
| $w_k,\ Q_k$    | $n\times1,\ n\times n$ | 过程噪声及其协方差 $E[w_kw_k^T]=Q_k$           |
| $v_k,\ R_k$    | $m\times1,\ m\times m$ | 测量噪声及其协方差 $E[v_kv_k^T]=R_k$           |

通常假设噪声零均值，且 $w_k$、$v_k$ 与相关的先验估计误差互不相关；以下协方差公式依赖这些假设。$v_k$ 是噪声向量，不要与状态中的速度混淆。$Q$ 越大表示越不信任运动模型，$R$ 越大表示越不信任测量；它们的具体数值应依据误差统计与实际数据确定。

对匀速位置—速度状态 $x=[p,\dot p]^T$，采样间隔 $\Delta t$ 下

$$
F=\begin{bmatrix}1&\Delta t\\0&1\end{bmatrix},\qquad
H=\begin{bmatrix}1&0\end{bmatrix}
$$

表示位置按速度外推，而传感器只量到位置。若用随机加速度描述模型误差，$Q$ 可由加速度噪声经动力学离散化得到，而不必简单设成单位阵。

### 4.4 矩阵 KF 的推导

**预测。** 把上一帧更新后的状态送入运动模型：

$$
\boxed{\hat x_{k|k-1}=F_k\hat x_{k-1|k-1}}.
$$

由于预测误差 $e_{k|k-1}=F_ke_{k-1|k-1}+w_k$，并假设交叉协方差为零，

$$
\boxed{P_{k|k-1}=F_kP_{k-1|k-1}F_k^T+Q_k}.
$$

这也解释了为什么右侧是 $P_{k-1|k-1}$：预测从上一帧**已经融合完上一帧测量**的结果出发。

**创新。** 预测测量为 $\hat z_{k|k-1}=H_k\hat x_{k|k-1}$，实际测量与它的差为

$$
y_k=z_k-H_k\hat x_{k|k-1}.
$$

由 $y_k=H_k(x_k-\hat x_{k|k-1})+v_k$ 可得

$$
\boxed{S_k=\operatorname{Cov}(y_k)=H_kP_{k|k-1}H_k^T+R_k}.
$$

$H_kP_{k|k-1}H_k^T$ 是**状态预测误差投影到观测空间后的协方差**，维度 $m\times m$；加上测量噪声才得到创新协方差 $S_k$。在标量例子中，它们分别为 $4$、$9$，因此 $S=13$。矩阵中还包含观测量之间的相关性，不能只看一个“大小”。

**增益。** 令 $\hat x_{k|k}=\hat x_{k|k-1}+K_ky_k$，其中 $K_k\in\mathbb R^{n\times m}$。其误差协方差为

$$
P_{k|k}(K)=(I-KH)P^-(I-KH)^T+KRK^T,
$$

此处为简洁省略 $k$ 下标，$P^-=P_{k|k-1}$。展开并对迹 $\operatorname{tr}P_{k|k}(K)$ 关于 $K$ 求导，得到

$$
-2P^-H^T+2K(HP^-H^T+R)=0,
$$

故在 $S$ 可逆时，

$$
\boxed{K_k=P_{k|k-1}H_k^TS_k^{-1}},\qquad
\boxed{\hat x_{k|k}=\hat x_{k|k-1}+K_ky_k}.
$$

工程上通常通过线性方程组求 $K$，避免显式计算逆矩阵。协方差推荐用数值性质更稳健的 Joseph 形式更新：

$$
\boxed{P_{k|k}=(I-K_kH_k)P_{k|k-1}(I-K_kH_k)^T+K_kR_kK_k^T}.
$$

在精确算术和最优增益下，它等价于 $(I-K_kH_k)P_{k|k-1}$。注意 $S$ 增大不能脱离原因直接断言“增益变小”：若增大的是 $R$，通常更少相信测量；若增大的是 $P^-$，标量情形下反而更依赖测量。

### 4.5 EKF：用函数预测，用雅可比传播误差

非线性模型写作 $x_k=f_k(x_{k-1})+w_k$、$z_k=g_k(x_k)+v_k$；这里用 $g$ 表示观测函数，避免与下文高度差 $h$ 重名。围绕当前估计做一阶展开：

$$
f_k(\hat x+\delta x)\approx f_k(\hat x)+F_k\delta x,
\qquad F_k=\left.\frac{\partial f_k}{\partial x}\right|_{\hat x_{k-1|k-1}},
$$

$$
g_k(\hat x^-+\delta x)\approx g_k(\hat x^-)+H_k\delta x,
\qquad H_k=\left.\frac{\partial g_k}{\partial x}\right|_{\hat x_{k|k-1}}.
$$

因此 EKF 使用

$$
\begin{aligned}
\hat x_{k|k-1}&=f_k(\hat x_{k-1|k-1}), &P_{k|k-1}&=F_kP_{k-1|k-1}F_k^T+Q_k,\\
y_k&=z_k-g_k(\hat x_{k|k-1}), &S_k&=H_kP_{k|k-1}H_k^T+R_k,\\
K_k&=P_{k|k-1}H_k^TS_k^{-1}, &\hat x_{k|k}&=\hat x_{k|k-1}+K_ky_k.
\end{aligned}
$$

$P_{k|k}$ 仍可按 Joseph 形式计算。**EKF 中 $H_k$ 只把小的状态误差 $\delta x$ 局部映射为观测误差 $\delta z$；一般不能写 $\hat z=H_k\hat x$，必须用非线性函数 $g_k(\hat x)$。** 同理，EKF 的 $F_k$ 是 $f_k$ 的雅可比，均值预测仍使用 $f_k$。一阶近似只在估计附近可靠；角度残差要按周期归一化，错误的数据关联和过大的线性化误差会破坏估计。

### 4.6 RoboMaster 四装甲板：从板位姿到整车状态

#### 4.6.1 明确几何和坐标

取车体坐标系原点为四板水平投影的对称中心，$x_b,y_b$ 为相互垂直的水平轴，$z_b$ 向上。$A_1,A_3$ 为第一对相对装甲板，$A_2,A_4$ 为第二对。定义 $d_1,d_2>0$ 为**各对中心在水平面内的间距**，$r=d_1-d_2$ 为带符号差；若两对高度不同，不能把板中心三维欧氏距离直接当作这里的 $d_i$。

令 $\bar d=(d_1+d_2)/2$，则

$$
d_1=\bar d+\frac r2,\qquad d_2=\bar d-\frac r2,
\qquad d_1,d_2>0.
$$

设第一对的板中心高度为 $z_c+\eta_1$，第二对为 $z_c+\eta_2$。取 $z_c$ 为两组高度的平均，$h=\eta_1-\eta_2$，于是 $\eta_1=h/2$、$\eta_2=-h/2$。四个板中心在车体系中的位置为

$$
\begin{aligned}
b_1&=[d_1/2,\ 0,\ h/2]^T,&b_3&=[-d_1/2,\ 0,\ h/2]^T,\\
b_2&=[0,\ d_2/2,\ -h/2]^T,&b_4&=[0,\ -d_2/2,\ -h/2]^T.
\end{aligned}
$$

这里假设每对板内部等高，且两对共享一个水平对称中心；真实结构若不满足，应增加偏移参数。$r$ 表示**两对间距之差**，并非某块板到中心的半径，也不是“同一对内部两块板”的距离差。仅凭双轴对称和 $r$ 无法确定绝对尺寸：还须已知 $\bar d$，或分别估计 $d_1,d_2$。例如 $d_1=0.40\,\mathrm m$、$d_2=0.30\,\mathrm m$ 时 $\bar d=0.35\,\mathrm m$、$r=0.10\,\mathrm m$，两组半径分别为 $0.20$ 和 $0.15\,\mathrm m$。

#### 4.6.2 运动模型、观测模型及维度

若 $\bar d$ 已由机械图纸或可靠标定给定，可取一个 $10$ 维状态

$$
X=[c_x,c_y,c_z,v_x,v_y,v_z,\psi,\omega,r,h]^T\in\mathbb R^{10}.
$$

用常速度、常 yaw 角速度作预测：$c_k=c_{k-1}+v_{k-1}\Delta t$，$v_k=v_{k-1}$，$\psi_k=\psi_{k-1}+\omega_{k-1}\Delta t$，$\omega_k=\omega_{k-1}$，$r_k=r_{k-1}$，$h_k=h_{k-1}$，再加入过程噪声。因此 $F_k\in\mathbb R^{10\times10}$，其非零项除单位阵外为 $F_{c_x,v_x}=F_{c_y,v_y}=F_{c_z,v_z}=F_{\psi,\omega}=\Delta t$。$r,h$ 对同一台刚性机器人通常近似常量；设很小的过程噪声可容纳标定误差，却不意味着它们物理上每帧变化。

设世界系车体中心 $C=[c_x,c_y,c_z]^T$，yaw 旋转矩阵 $R_z(\psi)$。板 $i$ 的世界系中心位置

$$
p_i^w=C+R_z(\psi)b_i.
$$

若视觉/PnP 输出板中心的世界系三维位置，则每块板的观测函数可取 $g_i(X)=p_i^w\in\mathbb R^3$；看到 $M$ 块时堆叠成 $g(X)\in\mathbb R^{3M}$，对应 $H\in\mathbb R^{3M\times10}$、$R\in\mathbb R^{3M\times3M}$。世界坐标需由相机外参正确转换。

若直接使用像素中心，则先变换到相机系 $p_i^c=R_{cw}(p_i^w-t_{wc})$，再用针孔模型（实际还需处理畸变）

$$
g_i(X)=\pi(p_i^c)=
\begin{bmatrix}f_xX_i^c/Z_i^c+c_x^{\mathrm{cam}}\\
f_yY_i^c/Z_i^c+c_y^{\mathrm{cam}}\end{bmatrix},\qquad Z_i^c>0.
$$

此时 $g\in\mathbb R^{2M}$、$H\in\mathbb R^{2M\times10}$、$R\in\mathbb R^{2M\times2M}$。相机内外参、相机系约定及板编号都须与模型一致。像素中心未包含板朝向信息；若检测/PnP 还提供姿态，可以在观测中增加朝向项，并相应处理角度残差。

以世界系位置观测 $A_1$ 为例，记 $e_x=[1,0,0]^T$、$e_z=[0,0,1]^T$，则

$$
g_1(X)=C+\frac{d_1}{2}R_z(\psi)e_x+\frac h2e_z,
$$

$$
\frac{\partial g_1}{\partial C}=I_3,\quad
\frac{\partial g_1}{\partial\psi}=\frac{d_1}{2}R_z'(\psi)e_x,\quad
\frac{\partial g_1}{\partial r}=\frac14R_z(\psi)e_x,\quad
\frac{\partial g_1}{\partial h}=\frac12e_z.
$$

对速度和 $\omega$ 的**直接**观测雅可比列为零；它们仍可通过 $F$ 与跨帧运动影响后续观测。像素观测的雅可比再用链式法则左乘投影雅可比 $\partial\pi/\partial p_i^c$ 及相机旋转。

若已知 $d_1$、$\psi$ 且量到 $A_1$ 的世界系中心，可反推 $C=p_1^w-R_z(\psi)b_1$；这里还需知道该板属于哪一组及其高度偏移。把单块板的 PnP 位姿直接当作整车中心，会遗漏这些几何与姿态条件。

#### 4.6.3 可观测性与建模选择

单帧只看到一块板的位置时，$C$ 与该板的半径或高度偏移可互相补偿。例如 $g_1=C+\tfrac{d_1}{2}R_z(\psi)e_x+\tfrac h2e_z$，在 $\psi$ 固定时，改变 $d_1$ 并反向移动 $C$ 可以保持相同的观测。因此“雅可比某列非零”并不等于该参数可单独估计；单块板中心也不足以唯一求出整车 yaw、两组尺寸和高度差。即使连续跟踪同一块板，若运动缺少有效激励、中心轨迹又自由变化，几何参数仍可能与中心运动耦合。

当同帧识别出相对的两块板，且取得统一坐标系下的三维中心位置时，理想模型给出 $C_{xy}=(p_1^w+p_3^w)_{xy}/2$、$d_1=\|(p_1^w-p_3^w)_{xy}\|$；另一对同理。识别两组还能得到 $r=d_1-d_2$ 和 $h=z_{1,3}-z_{2,4}$。多帧不同朝向的可靠观测也可能提供足够信息，但是否可观测要结合具体传感器、已知先验、数据关联及沿轨迹堆叠的雅可比秩来检验。只有单目像素中心、没有尺度来源时，绝对长度还存在尺度歧义；带已知尺寸的板角点/PnP 可提供尺度，但测量误差及姿态歧义仍需处理。

实际设计可二选一：已知 $\bar d$ 时把 $r,h$ 作为慢变化状态；未知绝对尺寸时改用 $d_1,d_2,h$（状态为 $11$ 维），最后输出 $r=d_1-d_2$。如果几何已标定且固定，也可把它们作为常量，只滤位置、速度和 yaw，减少弱可观测状态。

### 4.7 一帧流程速查

1. 用 $f$ 预测 $\hat X^-$，用 $F$ 和 $Q$ 传播 $P^-$。
2. 根据本帧可见且已正确关联的装甲板，计算 $g(\hat X^-)$ 和对应雅可比 $H$；构造匹配该观测维度的 $R$。
3. 计算创新 $y=z-g(\hat X^-)$、创新协方差 $S=HP^-H^T+R$，必要时用 $y^TS^{-1}y$ 检查异常观测。
4. 求 $K=P^-H^TS^{-1}$，更新 $\hat X^+=\hat X^-+Ky$，归一化 yaw，并以 Joseph 形式更新 $P^+$。
5. 无有效观测时只做预测；连续若干帧后检查创新统计和可观测性，再调 $Q,R$ 与几何先验。

一句话记忆：$F$ 管**状态随时间怎样变**，$H$ 管**小的状态误差怎样影响观测**，$P$ 表示目前有多不确定，$Q$ 和 $R$ 分别描述运动模型与测量的噪声；$HPH^T+R$ 衡量这次预测与测量的差异应有多大。

---

## 五、第三阶段作业

见 [`homework_3.md`](homework_3.md)：

- **第一部分**：ROS2 装甲板识别节点（本阶段第一节的内容）
- **第二部分**：位姿解算与弹道解算

---

## 六、参考资料

**ROS2**

- 官方教程：https://docs.ros.org/en/humble/Tutorials.html
- 话题与 QoS：https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
- 自定义消息（`rosidl`）：https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html
- `cv_bridge`（`vision_opencv`）：https://github.com/ros-perception/vision_opencv

**坐标系与解算（待补）**

- OpenCV 相机标定与 PnP：https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html

> 注：也可以适当参考 AI 进行学习，但要注意信息甄别，而且不要只让 AI 全程完成项目。
