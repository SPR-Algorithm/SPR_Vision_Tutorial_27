# OpenCV 培训简介

> 面向零基础：这篇只讲**传统图像处理**，一行深度学习都不碰。
> 配套可运行代码：[`opencv_basics.cpp`](opencv_basics.cpp)（C++）、[`opencv_basics.py`](opencv_basics.py)（Python）
> 完整实战：[`armor_detect.cpp`](armor_detect.cpp)（把下面这些 API 串成装甲板识别）

---

## 0. 这份材料怎么用

| 你想干什么            | 怎么做                                                                   |
| --------------------- | ------------------------------------------------------------------------ |
| 先看效果              | 跑一遍 `opencv_basics`，终端出数字、目录里出三张拼图                     |
| 边看边学              | 本文每一节的结构都是「概念 → API → 最小代码 → 效果/经验 → 自瞄里怎么用」 |
| 快速调参              | 用 Python 版，改完直接跑，不用重新编译                                   |
| 看这些 API 怎么串起来 | 看 `armor_detect.cpp` 和本文第 13 节                                     |

```bash
# C++
cd example/opencv
cmake -S . -B build && cmake --build build
./build/opencv_basics                  # 默认读 demo/bule_armoe.jpg
./build/opencv_basics demo/blue_5.png  # 换成自己的图

# Python（不用编译，调参首选）
pip install opencv-python numpy
python3 opencv_basics.py

# 服务器 / 容器里没有图形界面：加 NO_WINDOW=1，只存图不开窗
NO_WINDOW=1 ./build/opencv_basics
```

跑完会得到三张拼图：`opencv_basics_1_filter.png`（滤波 + 二值化）、`opencv_basics_2_edge.png`（形态学 + 边缘）、`opencv_basics_3_misc.png`（颜色 + 轮廓 + 变换 + 绘制）。

---

## 1. OpenCV 是什么

- **OpenCV（Open Source Computer Vision Library）** 是一个跨平台的图像 / 视频处理库，主接口是 C++，另有 Python、Java 绑定。
- 我们只用它的**传统图像处理**能力（阈值、滤波、轮廓、几何变换），不碰 `dnn` 模块的深度学习推理。

**模块地图**（记不住没关系，用到哪个查哪个）：

| 模块        | 头文件                  | 干什么                              | 我们会用吗     |
| ----------- | ----------------------- | ----------------------------------- | -------------- |
| `core`      | `opencv2/core.hpp`      | `cv::Mat`、矩阵运算、`Scalar/Point` | ✅ 到处都在用   |
| `imgproc`   | `opencv2/imgproc.hpp`   | 滤波、二值化、形态学、边缘、轮廓    | ✅ 主战场       |
| `imgcodecs` | `opencv2/imgcodecs.hpp` | `imread` / `imwrite`                | ✅              |
| `highgui`   | `opencv2/highgui.hpp`   | `imshow` / `waitKey`（调试用）      | ✅ 调试         |
| `videoio`   | `opencv2/videoio.hpp`   | `VideoCapture` 读摄像头 / 视频      | 第三阶段会用   |
| `calib3d`   | `opencv2/calib3d.hpp`   | 相机标定、`solvePnP` 位姿解算       | 第三阶段会用到 |
| `dnn`       | `opencv2/dnn.hpp`       | 跑 ONNX 模型                        | ❌ 本阶段不用   |

> 工程里图省事就 `#include <opencv2/opencv.hpp>`，把常用模块一次拉进来。

**装环境**：

```bash
# Ubuntu 22.04（课程环境）
sudo apt update && sudo apt install -y build-essential cmake libopencv-dev pkg-config
pkg-config --modversion opencv4          # 22.04 自带 4.5.4

# macOS
brew install opencv

# Python（哪个平台都行）
pip install opencv-python numpy
```

---

## 2. `cv::Mat`：绕不过去的数据结构

**一句话**：`cv::Mat` = 带着元信息的二维数组。图像处理 90% 的错误都来自对它的三个误解。

```cpp
#include <opencv2/opencv.hpp>

cv::Mat img = cv::imread("armor.jpg");   // 默认读成 BGR 三通道
if (img.empty()) return 1;               // ⚠ 读失败不抛异常，只返回空 Mat

img.rows;          // 高（先行！）
img.cols;          // 宽
img.channels();    // 通道数：灰度 1、彩色 3
img.type();        // 元素类型，如 CV_8UC3 = 8 位无符号 + 3 通道
img.at<cv::Vec3b>(y, x);   // 取像素：先 y 后 x
cv::Mat deep = img.clone();  // 深拷贝，独立一份数据
cv::Mat roi  = img(cv::Rect(100, 100, 200, 200));  // ROI，共享内存、不复制
```

| 坑                       | 现象与解法                                                                   |
| ------------------------ | ---------------------------------------------------------------------------- |
| **BGR 不是 RGB**         | `imread` 读进来就是 BGR。红色是 `(0,0,255)`、蓝色是 `(255,0,0)`，别按 RGB 想 |
| **`Mat b = a` 是浅拷贝** | 引用计数共享同一块数据，改 `b` 会影响 `a`。要独立副本用 `a.clone()`          |
| **行列顺序**             | `rows` = 高、`cols` = 宽；`at<T>(y, x)` 先 y 后 x                            |
| **`type()` 别硬编码**    | 灰度是 `CV_8UC1`、彩色是 `CV_8UC3`，写死了换成别的图就崩                     |

**Python 里没有 `cv::Mat`**，图像就是 `numpy.ndarray`：`shape = (高, 宽, 通道)`，同样也是 BGR。坐标一律 `img[y, x]`。

```python
img = cv2.imread("armor.jpg")     # 失败返回 None
h, w, c = img.shape               # 1706, 1279, 3
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)   # shape 变 (1706, 1279)
```

`opencv_basics` 实测输出：

```
  rows×cols  1706 × 1279（行=高，列=宽）
  通道数     3
  类型       CV_8UC3
  每行字节   3837（连续内存：是）
```

---

## 3. 读图、显示、存图

| API                       | 作用                          | 关键参数 / 返回                               |
| ------------------------- | ----------------------------- | --------------------------------------------- |
| `cv::imread(path, flags)` | 读图                          | `IMREAD_GRAYSCALE` 直接读灰度；失败返回空 Mat |
| `cv::imwrite(path, img)`  | 存图                          | 按扩展名决定格式，返回 `bool`                 |
| `cv::imshow(win, img)`    | 开窗显示                      | 必须在主线程调用                              |
| `cv::waitKey(ms)`         | 等按键，**同时驱动 GUI 刷新** | `0` = 一直等；返回值是键码                    |

```cpp
cv::Mat img = cv::imread("armor.jpg");
if (img.empty()) { std::cerr << "读图失败\n"; return 1; }

cv::imshow("origin", img);
int key = cv::waitKey(0);            // 返回按下的键，'q' = 113
cv::imwrite("result.png", img);      // 存图
```

**经验**

- `imshow` + `waitKey` **只用来调试**，别放进正式流程；无显示环境（服务器、容器）里调用会崩 → 用环境变量 `NO_WINDOW=1` 关掉，改成 `imwrite` 存图看。
- 只想看灰度图 → `cv::imread(path, cv::IMREAD_GRAYSCALE)`，比读完再转灰度少一步。
- 中文路径在某些平台会失败，尽量用英文路径。

---

## 4. 颜色空间与通道

| API                                            | 作用                                     |
| ---------------------------------------------- | ---------------------------------------- |
| `cv::cvtColor(src, dst, code)`                 | 颜色空间转换，`code` 如 `COLOR_BGR2GRAY` |
| `cv::split(src, {b, g, r})` / `cv::merge(...)` | 拆通道 / 合通道                          |
| `cv::inRange(src, lower, upper, dst)`          | 按范围取掩码（常配 HSV 用）              |

**为什么要有 HSV**：BGR 里颜色和亮度混在一起，光照一变阈值就废；HSV 把**色相 H** 单独拆出来，筛颜色稳得多。

```cpp
cv::Mat hsv;
cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);   // H:0~180  S:0~255  V:0~255

// 红色横跨 0 度，必须分两段再合并
cv::Mat m1, m2, mask;
cv::inRange(hsv, cv::Scalar(0,   100, 100), cv::Scalar(10,  255, 255), m1);
cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), m2);
cv::bitwise_or(m1, m2, mask);

cv::Mat red_only;
cv::bitwise_and(img, img, red_only, mask);   // 用掩码把原图抠出来
```

> ⚠ `inRange` 的上下界参数类型是 `InputArray`，**花括号不能自动推成 `Scalar`**，必须写全 `cv::Scalar(...)`（这是本示例实际踩过的编译错误）。

`opencv_basics` 实测（`demo/bule_armoe.jpg` 是一块**蓝色**装甲板）：

```
  red 掩码白像素 0.35%   blue 掩码白像素 20.68%
```

红色掩码几乎是空的 —— 正好说明**色相筛得准**：这张图里根本没有红色目标。看拼图 `_3_misc.png` 的 `blue mask` 一格，两条灯条被干净地分离出来了。

---

## 5. 滤波：去噪，但要决定「保不保边」

| API                                                        | 特点                       | 适合                     | 速度 |
| ---------------------------------------------------------- | -------------------------- | ------------------------ | ---- |
| `cv::blur(src, dst, ksize)`                                | 邻域均值                   | 快速糊一下               | 最快 |
| `cv::GaussianBlur(src, dst, ksize, sigmaX)`                | 高斯加权，离得越远权重越小 | 通用去噪，**默认首选**   | 快   |
| `cv::medianBlur(src, dst, ksize)`                          | 取中位数                   | **椒盐噪声**、小亮点     | 中   |
| `cv::bilateralFilter(src, dst, d, sigmaColor, sigmaSpace)` | 空间近 + 颜色像才算权重    | 要保边（如保留灯条边界） | 慢   |

```cpp
cv::Mat f_blur, f_gauss, f_median, f_bilateral;
cv::blur(gray, f_blur, {5, 5});                      // ksize 必须是正奇数
cv::GaussianBlur(gray, f_gauss, {5, 5}, 0);          // sigmaX=0 → 由 ksize 自动推
cv::medianBlur(gray, f_median, 5);                   // 这里是 int，不是 Size
cv::bilateralFilter(gray, f_bilateral, 9, 75, 75);   // d=邻域直径
```

`opencv_basics` 用「与原图的平均绝对差」量化抹除强度（越大说明平滑越狠）：

```
    blur 5x5                 mean|diff|= 1.70
    GaussianBlur 5x5         mean|diff|= 1.08
    medianBlur 5             mean|diff|= 1.18
    bilateralFilter          mean|diff|= 1.84
```

**经验**

- **核尺寸（ksize）越大越糊**，一般从 `3` 或 `5` 起步；`ksize` 必须是**大于 1 的奇数**。
- 别指望用"标准差变小"来验证滤波效果：整图对比度高时 std 几乎不变（实测 66.9 → 66.4）。看**差分图**或拼图更直观。
- 自瞄里的顺序通常是：**先高斯去噪 → 再二值化 → 再用形态学清理**，双边滤波很少用（太慢，实时帧率扛不住）。

---

## 6. 二值化：把灰度图变成黑白

| API                                                                     | 作用                                | 什么时候用                  |
| ----------------------------------------------------------------------- | ----------------------------------- | --------------------------- |
| `cv::threshold(src, dst, thresh, maxval, cv::THRESH_BINARY)`            | 固定阈值                            | 光照稳定、已知灰度分布      |
| `cv::threshold(src, dst, 0, 255, cv::THRESH_BINARY \| cv::THRESH_OTSU)` | **OTSU 自动找阈值**，返回值就是阈值 | 双峰直方图（前景/背景分明） |
| `cv::adaptiveThreshold(src, dst, 255, method, type, blockSize, C)`      | 每个像素用它邻域的阈值              | 光照严重不均（阴影、补光）  |

```cpp
cv::Mat b_fixed, b_otsu, b_adapt;
cv::threshold(gray, b_fixed, 150, 255, cv::THRESH_BINARY);            // 手填 150，换光照就废
double t = cv::threshold(gray, b_otsu, 0, 255,
                         cv::THRESH_BINARY | cv::THRESH_OTSU);        // 返回值 = OTSU 算出的阈值
cv::adaptiveThreshold(gray, b_adapt, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                      cv::THRESH_BINARY, 31, 5);                       // blockSize 31，C = 5
```

`opencv_basics` 实测：

```
  OTSU 自动算出的阈值 = 113（本次固定阈值用的是 150）
  白像素占比：固定 27.51%   OTSU 39.13%   自适应均值 80.72%   自适应高斯 86.54%
```

**经验**

- **OTSU 比手填阈值稳**：这张图 OTSU 自己算出 113，而手填 150 把大片背景也吞进来了（看拼图 `_1_filter.png` 里 `threshold 150` 一格，顶部一片黑）。
- **自适应阈值不是越智能越好**：它在平坦区域也硬要分个黑白，结果白像素占比冲到 80%+，噪点爆炸。只有**光照极不均匀**时才用它。
- `adaptiveThreshold` 的 `blockSize` 也是**大于 1 的奇数**，`C` 是从邻域均值里减掉的常数（越大越"严格"，白区越小）。
- 想按**颜色**分割？别用 `threshold`，用第 4 节的 `inRange`。

---

## 7. 形态学：在二值图上"修形状"

| API / 操作                                  | 效果                      | 自瞄里用来         |
| ------------------------------------------- | ------------------------- | ------------------ |
| `cv::erode`                                 | 腐蚀，白区变小            | 断开粘连           |
| `cv::dilate`                                | 膨胀，白区变大            | 补断点             |
| `cv::morphologyEx(..., cv::MORPH_OPEN)`     | 先腐蚀后膨胀 = **开运算** | **去孤立小噪点**   |
| `cv::morphologyEx(..., cv::MORPH_CLOSE)`    | 先膨胀后腐蚀 = **闭运算** | 补内部空洞         |
| `cv::morphologyEx(..., cv::MORPH_GRADIENT)` | 膨胀 − 腐蚀               | 快速拿轮廓         |
| `cv::morphologyEx(..., cv::MORPH_TOPHAT)`   | 原图 − 开运算             | 提取比背景亮的小块 |

```cpp
cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});  // 还有 ELLIPSE / CROSS
cv::Mat opened, closed;
cv::morphologyEx(bin, opened, cv::MORPH_OPEN,  kernel);   // 去噪点
cv::morphologyEx(bin, closed, cv::MORPH_CLOSE, kernel);   // 补空洞
```

`opencv_basics` 实测（看绝对像素数，比看百分比清楚）：

```
  白像素个数：OTSU 853776 → 开运算 848925（少了 4851 个孤立噪点） → 闭运算 856494（多了 2718 个补上的空洞）
```

**经验**

- **结构元形状跟着目标形状走**：灯条是竖直的细长条 → 用 `MORPH_RECT` 的 `{3,5}` 之类；圆形目标用 `MORPH_ELLIPSE`。
- 开运算去噪会**同时磨细目标**，"去噪"和"保目标"是一对矛盾，核别开太大。
- 迭代次数是 `morphologyEx` 最后一个参数，等价于连续做 N 次，效果比放大核更"狠"。

---

## 8. 边缘检测

| API                                          | 类型     | 特点                                      |
| -------------------------------------------- | -------- | ----------------------------------------- |
| `cv::Sobel(src, dst, CV_16S, dx, dy, ksize)` | 一阶方向 | 能分别拿横/竖边缘，`dx=1,dy=0` 是竖向边缘 |
| `cv::Laplacian(src, dst, CV_16S, ksize)`     | 二阶     | 对噪声敏感，一般先滤波                    |
| `cv::Canny(src, dst, low, high)`             | 双阈值   | **最常用**，输出干净的单像素边缘          |

```cpp
cv::Mat sx, sy, sxy, lap, edges;
cv::Sobel(gray, sx, CV_16S, 1, 0, 3);   // ⚠ 输出是 CV_16S，有正有负
cv::convertScaleAbs(sx, sx);            // 取绝对值才能当图看
cv::Sobel(gray, sy, CV_16S, 0, 1, 3);
cv::convertScaleAbs(sy, sy);
cv::addWeighted(sx, 0.5, sy, 0.5, 0, sxy);   // 两方向合起来 ≈ 梯度幅值

cv::Canny(gray, edges, 50, 150);     // low : high ≈ 1 : 2 ~ 1 : 3
```

`opencv_basics` 实测：

```
  平均梯度强度：Sobel-|dx| 13.3  Sobel-|dy| 11.6  Laplacian 11.5
  二值边缘（Canny）像素占比：阈值(50,150) → 3.04%   阈值(100,200) → 1.62%
```

**经验**

- **Sobel / Laplacian 的输出类型要自己选**：`CV_16S` 保精度（再 `convertScaleAbs`），`CV_8U` 会截断负值。
- **Canny 的两个阈值一起调**：低阈值决定"连不连得上"，高阈值决定"算不算强边缘"。阈值翻倍，边缘像素几乎减半（3.04% → 1.62%）。
- Canny 内部自带高斯平滑，别在外面又猛糊一遍，否则边缘全丢。
- 自瞄里 Canny 用得不多 —— 我们要的是"亮块位置"，二值化 + 轮廓更直接；Canny 更多用于标定、找靶纸。

---

## 9. 轮廓与几何量

| API                                             | 输入                     | 输出                                      |
| ----------------------------------------------- | ------------------------ | ----------------------------------------- |
| `cv::findContours(img, contours, mode, method)` | **二值图**（会被修改！） | `vector<vector<Point>>`，每个轮廓是一串点 |
| `cv::contourArea(c)`                            | 轮廓                     | 面积（像素²）                             |
| `cv::arcLength(c, true)`                        | 点列 + 是否闭合          | 周长                                      |
| `cv::boundingRect(c)`                           | 轮廓                     | 正矩形 `Rect`（快）                       |
| `cv::minAreaRect(c)`                            | 轮廓                     | **旋转矩形** `RotatedRect`（贴合斜目标）  |
| `cv::approxPolyDP(c, out, eps, true)`           | 轮廓                     | 多边形逼近，`eps` 是允许的偏差            |

```cpp
std::vector<std::vector<cv::Point>> contours;
cv::findContours(binary.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//  ⚠ RETR_EXTERNAL 只取最外层；findContours 会改输入图，所以先 clone

std::sort(contours.begin(), contours.end(),
          [](auto & a, auto & b) { return cv::contourArea(a) > cv::contourArea(b); });

for (const auto & c : contours) {
  const double area = cv::contourArea(c);
  if (area < min_area) continue;              // 面积筛：面积阈值写成「占整图的比例」

  const cv::Rect box = cv::boundingRect(c);    // 正矩形
  const cv::RotatedRect rr = cv::minAreaRect(c);  // 旋转矩形
  const double w = rr.size.width, h = rr.size.height;
  const double ratio = std::max(w, h) / std::min(w, h);   // 长宽比 → 用来认「灯条」
}
```

`opencv_basics` 实测（按面积排序的前 5 个轮廓）：

```
    #1 面积=528810 正矩形=1279×620 旋转矩形=619×1278 角度=-90.0° 长宽比=2.06
    #2 面积=170622 正矩形=449×696  旋转矩形=442×696  角度=-0.8°  长宽比=1.57
    #3 面积=22378  正矩形=199×214  旋转矩形=177×212  角度=-52.0° 长宽比=1.20
    #4 面积=18058  正矩形=88×376   旋转矩形=379×55   角度=-83.4° 长宽比=6.85   ← 灯条！
    #5 面积=16188  正矩形=84×365   旋转矩形=49×368   角度=-6.6°  长宽比=7.49   ← 灯条！
  findContours 共 78 个外轮廓，面积 ≥ 4364px 的留下 12 个
```

**这就是装甲板识别最关键的一步**：`#4` 和 `#5` 就是两根灯条 —— 旋转矩形给出的尺寸是 `379×55` / `49×368`，**长宽比 6.85 / 7.49**，和 `armor_detect.cpp` 里筛出来的灯条长宽比完全一致。剩下要做的只是「两两配对」。

**经验**

- **`findContours` 会修改输入图**（在旧版本上是直接擦除），养成 `binary.clone()` 的习惯。
- **`minAreaRect` 的返回值是 `RotatedRect`**：`(中心点, (宽, 高), 角度)`。角度约定在 **OpenCV 4.5 之后变过**（从 `[-90,0)` 变成 `(0,90]` 之类），跨版本移植时一定要重新验证，别照抄别人代码里的角度判断。
- **`boundingRect` 便宜、`minAreaRect` 贵**：能用正矩形快速粗筛，就别对每个轮廓都算旋转矩形。
- `RETR_EXTERNAL` 只给最外层轮廓（自瞄里够用）；`RETR_TREE` 会给完整层级关系（找嵌套图形时用）。
- **筛选阈值一律写成「相对整图尺寸的比例」**（`0.05f * img.rows`），换相机分辨率不用改参数 —— 这是本阶段反复强调的习惯。

---

## 10. 几何变换：改大小、转角、换视角

| API                                                                   | 作用                        |
| --------------------------------------------------------------------- | --------------------------- |
| `cv::resize(src, dst, size, fx, fy, interpolation)`                   | 缩放（缩小用 `INTER_AREA`） |
| `cv::getRotationMatrix2D(center, angle, scale)` + `warpAffine`        | 旋转 / 平移 / 缩放（仿射）  |
| `cv::getPerspectiveTransform(src_quad, dst_quad)` + `warpPerspective` | 透视变换（把斜的拉正）      |

```cpp
cv::Mat half;
cv::resize(img, half, {}, 0.5, 0.5, cv::INTER_AREA);   // 缩小用 AREA，放大用 LINEAR/CUBIC

// 旋转：先拿 2×3 的仿射矩阵，再 warpAffine
cv::Mat M = cv::getRotationMatrix2D({img.cols / 2.0f, img.rows / 2.0f}, 15.0, 1.0);  // 逆时针 15°
cv::Mat rotated;
cv::warpAffine(img, rotated, M, img.size());   // 第 4 个参数 = 输出图像尺寸

// 透视：把任意四边形"拉正"成矩形
std::vector<cv::Point2f> src_p = {{300, 400}, {900, 450}, {950, 1200}, {250, 1150}};
std::vector<cv::Point2f> dst_p = {{0, 0}, {400, 0}, {400, 300}, {0, 300}};
cv::Mat P = cv::getPerspectiveTransform(src_p, dst_p);
cv::Mat warped;
cv::warpPerspective(img, warped, P, {400, 300});
```

**经验**

- **缩小图像一定用 `INTER_AREA`**，否则会出现摩尔纹（锯齿状花纹）。
- `warpAffine` 的**第 4 个参数决定输出尺寸**：填 `img.size()` 时旋转会被裁掉四个角。想要"转完不裁"就得自己算外接矩形。
- 透视变换是后面**装甲板正畸 / 棋盘格矫正**的基础，四个点要按固定顺序（左上→右上→右下→左下）给。

---

## 11. 绘制：调试图必须画出来

| API                                                          | 画什么                     |
| ------------------------------------------------------------ | -------------------------- |
| `cv::line(img, p1, p2, color, thickness)`                    | 线段                       |
| `cv::arrowedLine(img, p1, p2, color, thickness)`             | 带箭头的线（画方向很有用） |
| `cv::rectangle(img, rect, color, thickness)`                 | 矩形（传 `Rect` 或两点）   |
| `cv::circle(img, center, radius, color, thickness)`          | 圆（`FILLED` = 实心）      |
| `cv::polylines(img, pts, isClosed, color, thickness)`        | 折线 / 多边形              |
| `cv::drawContours(img, contours, idx, color, thickness)`     | 批量画轮廓                 |
| `cv::putText(img, text, org, font, scale, color, thickness)` | 文字                       |

```cpp
cv::Mat canvas = img.clone();                       // ⚠ 别在原图上画（除非你就是想覆盖）
cv::rectangle(canvas, {60, 60, 300, 200}, {0, 255, 0}, 3);
cv::circle(canvas, {600, 300}, 40, {0, 0, 255}, cv::FILLED);
cv::arrowedLine(canvas, {900, 300}, {1100, 200}, {0, 255, 255}, 3);
cv::putText(canvas, "armor", {60, img.rows - 60},
            cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 255}, 2);
```

**经验**

- **坐标和线宽要随图尺寸缩放**：`scale = std::max(0.6, img.cols / 1200.0)`。写死 `thickness=2`，换到 4K 图上根本看不见。
- 颜色也是 `Scalar(B, G, R)` —— 又是 BGR，别写成 RGB。
- 调试图别只 `imshow`：**一定要 `imwrite` 存下来**，方便事后对比参数、放进报告。

---

## 12. 计时与性能

```cpp
const int64 t0 = cv::getTickCount();        // 不是毫秒，是"时钟滴答数"
// ... 干活 ...
const double ms = (cv::getTickCount() - t0) * 1000.0 / cv::getTickFrequency();
std::cout << ms << " ms\n";
```

`opencv_basics` 实测（1279×1706 的图，MacBook / OpenCV 5.0）：

```
  cvtColor(BGR2GRAY) 耗时 1.38 ms
  四种滤波合计 11.21 ms
  总耗时 432.56 ms      ← 10 组演示 + 拼图 + 存盘
```

**经验**

- **灰度化能省 2/3 的计算量**：后续所有处理都只在 1 个通道上做。
- 实时自瞄里 30fps = 每帧 33ms 预算，`bilateralFilter` 这种就别进循环了。
- 能用 ROI 就只处理 ROI，别对整张 4K 图跑轮廓。
- `cv::Mat b = a` 不复制数据，**但 `a.clone()` 会**；循环里频繁 clone 是常见的性能刺客。

---

## 13. 常见坑速查

| 坑                            | 症状                                            | 解法                                            |
| ----------------------------- | ----------------------------------------------- | ----------------------------------------------- |
| 忘了判空                      | `imread` 失败后一路崩                           | `if (img.empty()) return`                       |
| 以为 `Mat b = a` 是拷贝       | 改 b 把 a 也改了                                | 要独立数据用 `a.clone()`                        |
| 把 BGR 当 RGB                 | 红蓝互换，掩码全错                              | 记死：红 `(0,0,255)`、蓝 `(255,0,0)`            |
| `findContours` 改了输入图     | 后面再想用二值图发现被毁了                      | 传 `binary.clone()`                             |
| `inRange` 用花括号初始化      | 编译报 `no matching constructor for InputArray` | 写全 `cv::Scalar(...)`                          |
| 滤波核写成偶数                | 直接抛异常                                      | ksize 必须是**大于 1 的奇数**                   |
| 阈值写死像素数                | 换分辨率就失灵                                  | 一律写成「占整图尺寸的比例」                    |
| 相信 `minAreaRect` 的角度含义 | 跨 OpenCV 版本行为不一致                        | 换版本就重新验证，别硬套别人的角度判断          |
| `imshow` 在无显示环境崩       | 服务器上跑不动                                  | `NO_WINDOW=1` 只存图                            |
| **OpenCV 5 头文件搬家**       | `minAreaRect` 找不到                            | `__has_include(<opencv2/geometry/2d.hpp>)` 兼容 |

最后一条示例代码里是这么处理的：

```cpp
#include <opencv2/opencv.hpp>
#if __has_include(<opencv2/geometry/2d.hpp>)
#include <opencv2/geometry/2d.hpp>   // OpenCV 5 起 minAreaRect 等从 imgproc 挪到了这里
#endif
```

---

## 14. 从这些 API 到「装甲板识别」

把前面几节串起来，就是 `armor_detect.cpp` 的完整流水线：

```mermaid
flowchart LR
    A["① imread<br/>原图 BGR"] -->|cvtColor| B["② 灰度"]
    B -->|threshold OTSU| C["③ 二值图"]
    C -->|morphologyEx OPEN| D["④ 去噪"]
    D -->|findContours| E["⑤ 轮廓"]
    E -->|minAreaRect + 筛选| F["⑥ 候选灯条"]
    F -->|两两配对 + 打分| G["⑦ 装甲板四点"]
```

| 步骤          | 本文哪一节 | `armor_detect.cpp` 里的代码                             |
| ------------- | ---------- | ------------------------------------------------------- |
| 灰度 + 二值化 | 第 4、6 节 | `cvtColor` + `threshold(..., THRESH_OTSU)`              |
| 去噪          | 第 7 节    | `morphologyEx(..., MORPH_OPEN, ...)`                    |
| 找轮廓        | 第 9 节    | `findContours(..., RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)` |
| 灯条筛选      | 第 9 节    | `minAreaRect` → 长短 / 粗细 / **长宽比 1.5~15**         |
| 灯条配对      | —          | 间距 / 上下端对齐 / 高度相近 / 宽高比                   |
| 输出四点      | 第 10 节   | 固定顺序「左上 → 右上 → 右下 → 左下」                   |

**下一步**：这四个角点会被送进 `cv::solvePnP`（`calib3d` 模块），反推出装甲板在相机坐标系下的**位姿** —— 那是第三阶段的内容。

---

## 附：API 速查表

| 分类     | 常用 API                                                                                                |
| -------- | ------------------------------------------------------------------------------------------------------- |
| 读写显示 | `imread` `imwrite` `imshow` `waitKey` `VideoCapture`                                                    |
| 类型转换 | `cvtColor` `convertScaleAbs` `Mat::convertTo` `split` `merge`                                           |
| 滤波     | `blur` `GaussianBlur` `medianBlur` `bilateralFilter` `filter2D`                                         |
| 阈值     | `threshold`（`THRESH_BINARY` / `THRESH_OTSU` / `THRESH_TRIANGLE`） `adaptiveThreshold` `inRange`        |
| 形态学   | `erode` `dilate` `morphologyEx`（`OPEN` `CLOSE` `GRADIENT` `TOPHAT` `BLACKHAT`）`getStructuringElement` |
| 边缘     | `Sobel` `Scharr` `Laplacian` `Canny`                                                                    |
| 轮廓     | `findContours` `drawContours` `contourArea` `arcLength` `boundingRect` `minAreaRect` `approxPolyDP`     |
| 几何     | `resize` `warpAffine` `warpPerspective` `getRotationMatrix2D` `getPerspectiveTransform` `remap`         |
| 绘图     | `line` `arrowedLine` `rectangle` `circle` `polylines` `putText`                                         |
| 统计     | `countNonZero` `mean` `meanStdDev` `minMaxLoc` `sum`                                                    |
| 计时     | `getTickCount` `getTickFrequency`                                                                       |

> 查文档：https://docs.opencv.org/4.x/d7/dbd/group__imgproc.html
> 记不住函数名就搜"opencv + 想做的事"（如 `opencv rotate image`），官方文档永远排在前面。
