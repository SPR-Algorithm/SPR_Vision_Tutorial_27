# 第二阶段配套实例

`opencv/` 一个目录，覆盖教案的**算法部分**：先过一遍基础 API，再串成装甲板识别。

| 示例                    | 目录                                   | 对应教案              | 依赖   | 本机验证                                |
| ----------------------- | -------------------------------------- | --------------------- | ------ | --------------------------------------- |
| OpenCV · 基础 API 速览  | `opencv/opencv_basics.cpp` / `.py`     | 一、OpenCV（1.1~1.3） | OpenCV | ✅ C++ / Python 均已实跑通过            |
| OpenCV · 装甲板四点识别 | `opencv/`                              | 一、OpenCV（1.4）     | OpenCV | ✅ 已在 `demo/bule_armoe.jpg` 上实测通过 |

```
example/
└── opencv/
    ├── CMakeLists.txt
    ├── opencv_intro.md               培训简介：OpenCV 功能 + 常用 API + 调参经验
    ├── opencv_basics.cpp             基础 API 速览（C++），一次跑完 10 组 API
    ├── opencv_basics.py              同上，Python 版（调参更快）
    ├── armor_detect.cpp              传统方法识别装甲板，输出四个角点
    └── demo/
        ├── bule_armoe.jpg            演示图（蓝色灯条装甲板）
        ├── blue_5.png                演示图（蓝色 5 号装甲板）
        └── demo.mp4                  录像素材（544×960 / 30fps / 173s）
```

> **关于 `demo.mp4`**：示例只演示「单张图 → 装甲板四点」的算法本身。视频流部分在作业里做——
> 学员要写一个 `video_publisher` 把它逐帧发到 `/image_raw`。
>
> 注意视频是 **544 × 960**，和示例用的图（1279 × 1706）分辨率不同，参数要重调，详见 [`../homework_2.md`](../homework_2.md) §2.5。

---

## ① OpenCV · 基础 API 速览

先读 [`opencv_intro.md`](opencv_intro.md)（培训简介：每个 API 干什么、参数怎么调），
再跑 `opencv_basics` 把效果看一眼。这一步不做识别，只建立「哪个 API 干什么」的直觉。

```bash
cd example/opencv

# C++（走已有的 CMake 工程）
cmake -S . -B build && cmake --build build
./build/opencv_basics                     # 默认读 demo/bule_armoe.jpg
./build/opencv_basics demo/blue_5.png     # 换成自己的图
NO_WINDOW=1 ./build/opencv_basics         # 不开窗，只存图

# Python（不用编译，改完直接跑，调参首选）
pip install opencv-python numpy
python3 opencv_basics.py
```

一次跑完 10 组 API：读图/元信息 → 灰度化 → 滤波 → 二值化 → 颜色分割 → 形态学 → 边缘检测 → 轮廓 → 几何变换 → 绘制。
终端打印每一步的耗时与统计量，目录里留下三张对比拼图：

| 拼图                         | 内容                                               |
| ---------------------------- | -------------------------------------------------- |
| `opencv_basics_1_filter.png` | 4 种滤波 + 固定阈值 / OTSU / 自适应阈值            |
| `opencv_basics_2_edge.png`   | 4 种形态学 + Sobel / Laplacian / Canny             |
| `opencv_basics_3_misc.png`   | 颜色分割 + 轮廓 + 旋转 / 透视 + 绘制                 |

实测（`demo/bule_armoe.jpg`，1279×1706，OpenCV 5.0）节选：

```
【4】二值化 threshold / THRESH_OTSU / adaptiveThreshold
  OTSU 自动算出的阈值 = 113（本次固定阈值用的是 150）
  白像素占比：固定 27.51%   OTSU 39.13%   自适应均值 80.72%   自适应高斯 86.54%

【7】边缘检测 Sobel / Laplacian / Canny
  二值边缘（Canny）像素占比：阈值(50,150) → 3.04%   阈值(100,200) → 1.62%

【8】轮廓 findContours + 面积/外接矩形筛选
    #4 面积=18058 正矩形=88×376 旋转矩形=379×55 角度=-83.4° 长宽比=6.85   ← 灯条！
    #5 面积=16187 正矩形=84×365 旋转矩形=49×368 角度=-6.6° 长宽比=7.49   ← 灯条！
```

> 最后两行就是下一节要用的**灯条**：长宽比 6.85 / 7.49，和 `armor_detect` 筛出来的完全一致。

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

- `armor_detect.cpp` 里 `detectArmor()` 的输入是 `cv::Mat`、输出是四个点，**不依赖任何框架**。第三阶段（[`tutorial_3`](../../tutorial_3/tutorial_3.md)）会把它搬进 ROS2 节点：外面套一层「订阅 `/image_raw` → `cv_bridge` 转换 → 调用 → 发布 `/armor_detector/detections` + `/armor_detector/debug_image`」，算法本身一行不用改。
