# 大作业要求

自主完成一个在 ROS 2 仿真环境中进行 SLAM 建图和导航的程序。基础要求使用 Cartographer，进阶要求使用 FAST-LIO或Point-LIO。

## 提示（并非具体步骤）：

1. 创建一个简易的轮式机器人模型（URDF/Xacro），配置好 robot_state_publisher 与 joint_state_publisher（或仿真关节状态发布插件），确保拥有一棵完整的 TF 树（base_link 及各传感器坐标系）。

2. 在机器人模型上创建虚拟雷达和 IMU，作为 SLAM 和导航的数据来源。基础要求使用二维激光雷达；进阶要求使用 MID360 或等价三维激光雷达。

3. 搭建一个 Gazebo 环境容纳机器人，需体现环境特征（墙壁、柱子等），以便于雷达提取特征；配置轮式驱动，使机器人能够通过键盘控制运动。

4. 基础要求使用 Cartographer 建图，接入 Gazebo 轮式里程计；进阶要求使用 FAST-LIO或Point-LIO 作为里程计并建图。注意 TF 发布关系，避免 Gazebo 轮式里程计与建图算法重复发布同一段变换，确保 odom 到机器人底盘的变换唯一且准确。

5. 地图处理：
   - 基础要求：保存 Cartographer 生成的二维栅格地图（PGM/YAML），供 Navigation2 使用。
   - 进阶要求：保存 FAST-LIO或Point-LIO 生成的三维点云地图（PCD），并完成三维点云到二维栅格地图的转换或实时投影，可使用 OctoMap、pointcloud_to_laserscan 配合 slam_toolbox 或其他工具实现。

6. 安装并配置 Navigation2：
   - 如果使用 AMCL 定位，需提供二维激光扫描；使用三维雷达时，需解决三维点云转二维扫描的输入问题。
   - 如果使用 LIO 纯里程计盲跑（不推荐但可行），需处理 map 坐标系的初始对齐问题，并考虑累计漂移。

7. 最后目标：在同一工作空间中集成建图程序 A 和导航程序 B。
   - A 负责建图：启动仿真与建图，通过键盘节点控制机器人遍历环境，保存用于导航的二维地图；进阶还需保存三维地图。
   - B 负责导航：启动仿真、加载二维地图并完成定位（纯里程计方案须完成初始地图对齐），在 RViz2 中使用 Nav2 Goal 发布目标点，机器人能够自动规划路径、避开障碍并到达目标。

## 补充要求

1. 环境使用 Ubuntu 22.04 和 ROS 2 Humble。
2. 完成 Cartographer 方案即满足基础要求；完成 FAST-LIO或Point-LIO 方案为进阶要求。
3. 导航栈必须选择 Navigation2；机器人模型自定义，能够保证简单运动即可。
4. 学会Navigation2的配置，在提交完代码的工作后需要对于整个项目的数据流向和数据流出、以及整个工作空间中做一个word总结。（如果使用cartographer，需另交一份word说明FAST-LIO和Point-LIO在运行时的区别）
5. 代码当面验收，检查时将要求启动程序并测试导航功能。
6. 允许使用 AI 辅助，但需单独提交《AI 使用说明》，简述使用的工具、用途及涉及的代码或配置，并举例说明本人对 AI 输出的检查、修改和运行验证过程。未使用 AI 的注明“未使用”。验收时须能解释相关代码和配置。
7. 文档统一发送至2406310992@qq.com，写明姓名。
8. 截止日期为2026年11月7日。

## 参考内容

- [ROS 2 Humble 官方文档](https://docs.ros.org/en/humble/)
- [ROS 2 Humble 中文文档](https://ros2docs.robook.org/humble/index.html)
- [Navigation2 官方文档](https://docs.nav2.org/)
- [Navigation2 中文文档](http://dev.nav2.fishros.com/doc/index.html)
- [FAST-LIO 源码](https://github.com/hku-mars/FAST_LIO/tree/ROS2)
- [Point-LIO 源码](https://github.com/hku-mars/Point-LIO)
- [ROS2 机器人入门到实战](https://zhuanlan.zhihu.com/p/639234090)
