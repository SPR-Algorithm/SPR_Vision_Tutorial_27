自瞄的全部流程


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