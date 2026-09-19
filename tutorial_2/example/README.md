# 第二阶段配套实例

两个例子，覆盖教案的**算法部分**；ROS2 工程部分看[第二阶段作业](../homework_2.md)。

| 示例                    | 目录      | 对应教案         | 依赖     | 本机验证                                |
| ----------------------- | --------- | ---------------- | -------- | --------------------------------------- |
| 面向对象 · 相机类       | `oop/`    | 一、C++ 面向对象 | 只要 g++ | ✅ 已编译运行                            |
| OpenCV · 装甲板四点识别 | `opencv/` | 二、OpenCV       | OpenCV   | ✅ 已在 `demo/bule_armoe.jpg` 上实测通过 |

```
example/
├── oop/camera_class.cpp              抽象基类 + 多态 + 工厂（不依赖任何库）
└── opencv/
    ├── CMakeLists.txt
    ├── armor_detect.cpp              传统方法识别装甲板，输出四个角点
    └── demo/
        ├── bule_armoe.jpg            演示图（蓝色灯条装甲板）
        └── demo.mp4                  录像素材（544×960 / 30fps / 173s）
```

> **关于 `demo.mp4`**：示例只演示「单张图 → 装甲板四点」的算法本身。视频流部分在作业里做——
> 学员要写一个 `video_publisher` 把它逐帧发到 `/image_raw`。
>
> 注意视频是 **544 × 960**，和示例用的图（1279 × 1706）分辨率不同，参数要重调，详见 [`../homework_2.md`](../homework_2.md) §2.5。

---

## ① 面向对象 · 相机类

**不需要装任何东西**，一个 `g++` 就能跑：

```bash
cd example/oop
g++ -std=c++17 -Wall -Wextra camera_class.cpp -o camera
./camera
```

预期输出（节选）：

```
=== 1. 两种相机用同一段代码取图 ===
  [UsbCamera /dev/video0] open() 成功（模拟）
  UsbCamera(/dev/video0) 出图：640x480，灰度数据 307200 字节，首像素=0
  [UsbCamera /dev/video0] close()
  [IndustrialCamera SN20270101] open() 成功（模拟）
  IndustrialCamera(SN20270101) 出图：1280x1024，灰度数据 1310720 字节，首像素=0
  [IndustrialCamera SN20270101] close()

=== 2. 换成回放相机做回归测试，算法代码一行没改 ===
  [ReplayCamera demo.bag] open() 成功（模拟）
  ...

=== 3. 直接用引用调用（多态的最小验证）===
  UsbCamera(/dev/video1) 没有取到图      ← 没有 open 就读，read() 返回 false
```

**上课要指出的四个点**

1. `Camera` 是抽象基类，含纯虚函数，写 `Camera c;` 直接编译报错。
2. `runOnce(Camera &cam)` 里只有一句 `cam.read(frame)`，但运行期会分别调到 `UsbCamera::read` 和 `IndustrialCamera::read` —— 这就是多态。
3. 第 2 组加了一个 `ReplayCamera`，`runOnce()` **一行都没改**。这就是「算法与硬件解耦」的价值，也是第三阶段 `io` 层这么设计的原因。
4. 每个派生类的析构里都调了 `close()`，程序结束时自动执行 —— 顺带把 RAII 讲了。

**故意留的坑**：把 `virtual ~Camera() = default;` 的 `virtual` 去掉，第 1 组结尾的 `close()` 打印就会消失（派生类析构没被调用）。

## ② OpenCV · 装甲板四点识别

### 装依赖

```bash
# Ubuntu 22.04
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev pkg-config
pkg-config --modversion opencv4        # 22.04 自带 4.5.4
```

### 编译运行

```bash
cd example/opencv
cmake -S . -B build && cmake --build build

./build/armor_detect                       # 默认读 demo/bule_armoe.jpg
./build/armor_detect my_armor.png          # 也可以喂自己的图
DEBUG_PAIRS=1 ./build/armor_detect         # 打印所有候选配对及其得分
NO_WINDOW=1  ./build/armor_detect          # 不开窗口，只看终端和 result.png
```

### 实测输出（`demo/bule_armoe.jpg`，1279×1706）

```
已读入 demo/bule_armoe.jpg（1279x1706）

候选灯条 12 根：
  #0 中心(1137, 825) 高=379 宽=55 长宽比=6.85
  #1 中心(194, 816) 高=370 宽=49 长宽比=7.52
  ......

识别到装甲板：
  宽高比   2.64（理想值 2.5）
  灯条间距 987 像素
  灯条高度 370 / 379 像素
  四点坐标（顺序 = 左上 右上 右下 左下）：
    左上 P0 (  172.3,   632.8)
    右上 P1 ( 1159.0,   636.5)
    右下 P2 ( 1115.6,  1012.6)
    左下 P3 (  215.0,   999.9)
结果图已保存为 result.png
```

`result.png` 里：**绿色框 + P0~P3 四个角点**就是识别的最终输出，黄色是被筛选出来的候选灯条，红色是装甲板中心。

### 上课要指出的四个点

1. **这张图有三个陷阱**，正好逐个演示：
   - 装甲板上白色的数字「3」：灰度值比灯条还亮，长宽比也落进 1.5~15，靠 `bar.width > img.width * 0.08f` 这条**宽度约束**才挡掉。
   - 背景里的线束、高光：被长宽比和最短长度挡掉。
   - 画面顶部一条横向边缘：凑出了宽高比 2.37 的假配对，**比真装甲板的 2.64 还接近理想的 2.5**。

2. **加分权重是必需的**。用 `DEBUG_PAIRS=1` 可以看到打分对比：

   ```
   [候选] 左(172,633)h370  右(1159,637)h379 → 宽高比 2.64 得分 0.520   ← 真目标
   [候选] 左(953,-6)h112   右(1198,-5)h94   → 宽高比 2.37 得分 0.055   ← 假配对
   ```

   只比宽高比的话假配对会赢（-0.126 vs -0.138）。加 `kSizeWeight * (灯条高度 / 图高)` 之后差距才拉开。

3. **输出的四个点顺序固定为「左上 → 右上 → 右下 → 左下」**，这就是下一步做位姿解算（`solvePnP`）要喂进去的顺序。

4. **参数全是相对尺寸**（`0.05f`、`0.08f`、`2.5f` 这些都是比例），所以换相机分辨率不用改参数。这也正是作业要求「参数集中」的原因。

### 现场演示的调参实验

| 改动                                                          | 现象                             |
| ------------------------------------------------------------- | -------------------------------- |
| 把 `bar.width > img.width * 0.08f` 改成 `0.35f`（等价于放开） | 白色数字「3」混进候选灯条列表    |
| 把 `kSizeWeight` 改成 `0.0f`                                  | 识别框跑到画面顶部的假配对上去   |
| 把 `0.95f` 改回 `0.55f`                                       | 真装甲板被间距约束挡掉，识别失败 |
| 在 `detectArmor` 里删掉 `morphologyEx` 那一行                 | 候选灯条数量变多，噪点混进来     |

### 常见坑

- `cv::imread` 失败不报错，只返回空 `Mat`，一定要 `if (img.empty())`。
- OCR 之外最容易踩的：`cv::Mat b = a` 是**浅拷贝**，改 `b` 会影响 `a`。
- `findContours` 会修改输入图，习惯先 `clone()`。
- **OpenCV 5** 把 `minAreaRect` 从 `imgproc` 挪到了 `opencv2/geometry/2d.hpp`。示例里用 `__has_include` 做了兼容，4.x 和 5.x 都能编。

## 和后面阶段的衔接

- `oop/camera_class.cpp` 里的 `Camera` 抽象基类，就是第三阶段 `io` 层 `Camera / USBCamera` 的原型。
- `armor_detect.cpp` 里 `detectArmor()` 的输入是 `cv::Mat`、输出是四个点，**不依赖任何框架**。第二阶段作业就是把它搬进 ROS2 节点：外面套一层「订阅 `/image_raw` → `cv_bridge` 转换 → 调用 → 发布 `/armor_detector/detections` + `/armor_detector/debug_image`」，算法本身一行不用改。
