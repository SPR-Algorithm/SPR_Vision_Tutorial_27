// ============================================================================
// 知识点 1 · OpenCV —— 基础 API 速览（滤波 / 二值化 / 边缘 / 形态学 / 轮廓…）
//
//   cd example/opencv
//   cmake -S . -B build && cmake --build build
//   ./build/opencv_basics                     # 默认读 demo/bule_armoe.jpg
//   ./build/opencv_basics demo/blue_5.png     # 也可以喂自己的图
//   NO_WINDOW=1 ./build/opencv_basics         # 不开窗口，只存图
//
// 一次跑完最常用的几组 imgproc API，每一步都打印耗时 / 统计量：
//   ① 读图与 Mat 元信息   imread / rows·cols / type() / channels()
//   ② 灰度化             cvtColor(BGR2GRAY)
//   ③ 滤波去噪           blur / GaussianBlur / medianBlur / bilateralFilter
//   ④ 二值化             threshold / THRESH_OTSU / adaptiveThreshold
//   ⑤ 颜色分割           cvtColor(BGR2HSV) / inRange / bitwise_and
//   ⑥ 形态学             morphologyEx：开 / 闭 / 梯度 / 顶帽
//   ⑦ 边缘检测           Sobel / Laplacian / Canny
//   ⑧ 轮廓               findContours / contourArea / boundingRect /
//   minAreaRect ⑨ 几何变换           resize / getRotationMatrix2D + warpAffine
//   / 透视 ⑩ 绘制               line / rectangle / circle / putText
//
// 会存三张拼图：opencv_basics_1_filter.png / _2_edge.png / _3_misc.png
// ============================================================================

#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>
#if __has_include(<opencv2/geometry/2d.hpp>)
#include <opencv2/geometry/2d.hpp> // OpenCV 5 起把 minAreaRect 等从 imgproc 挪到了这里
#endif

namespace {

// ---------------------------------------------------------------------------
// 几个小工具（真正要讲的 API 在 main 里）
// ---------------------------------------------------------------------------

void section(int index, const std::string &title) {
  std::cout << "\n【" << index << "】" << title << "\n";
}

// 把一张图等比缩放后居中贴到固定尺寸的黑底上，方便拼图对齐
cv::Mat fit_into(const cv::Mat &src, const cv::Size &cell) {
  cv::Mat canvas(cell, CV_8UC3, cv::Scalar(0, 0, 0));

  cv::Mat bgr;
  if (src.channels() == 1)
    cv::cvtColor(src, bgr, cv::COLOR_GRAY2BGR);
  else
    bgr = src;

  const double s = std::min(static_cast<double>(cell.width) / bgr.cols,
                            static_cast<double>(cell.height) / bgr.rows);
  cv::Mat small;
  cv::resize(bgr, small, cv::Size(), s, s, cv::INTER_AREA);

  const cv::Rect roi((cell.width - small.cols) / 2,
                     (cell.height - small.rows) / 2, small.cols, small.rows);
  small.copyTo(canvas(roi));
  return canvas;
}

// 把若干「标题 + 图」拼成 cols 列的网格，一眼就能对比不同参数的效果
// 每格 = 上方 40px 标题条 + 下方 420×300 的图像区（标题不挡内容）
cv::Mat grid(const std::vector<std::pair<std::string, cv::Mat>> &items,
             int cols = 3) {
  const cv::Size area(420, 300);
  const int bar = 40;
  const cv::Size cell(area.width, area.height + bar);

  std::vector<cv::Mat> cells;
  cells.reserve(items.size());
  for (const auto &[title, img] : items) {
    cv::Mat c(cell, CV_8UC3, cv::Scalar(18, 18, 18));
    fit_into(img, area).copyTo(c(cv::Rect(0, bar, area.width, area.height)));
    cv::putText(c, title, {12, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                {0, 255, 255}, 2);
    cells.push_back(c);
  }
  while (cells.size() % cols != 0)
    cells.push_back(cv::Mat(cell, CV_8UC3, cv::Scalar(18, 18, 18)));

  std::vector<cv::Mat> rows;
  for (std::size_t i = 0; i < cells.size(); i += cols) {
    cv::Mat row;
    cv::hconcat(
        std::vector<cv::Mat>(cells.begin() + i, cells.begin() + i + cols), row);
    rows.push_back(row);
  }

  cv::Mat out;
  cv::vconcat(rows, out);
  return out;
}

// 矩阵元素类型名，例如 CV_8UC3
std::string type_name(int type) {
  static const char *depth_names[] = {"8U",  "8S",  "16U", "16S",
                                      "32S", "32F", "64F", "16F"};
  const int depth = CV_MAT_DEPTH(type);
  const std::string d = (depth >= 0 && depth < 8) ? depth_names[depth] : "?";
  return "CV_" + d + "C" + std::to_string(CV_MAT_CN(type));
}

// 灰度统计：均值 / 标准差。平滑越强，标准差越小
void stat_line(const std::string &name, const cv::Mat &m) {
  cv::Scalar mean, stddev;
  cv::meanStdDev(m, mean, stddev);
  std::cout << "    " << std::left << std::setw(24) << name << std::right
            << " mean=" << std::fixed << std::setprecision(1) << std::setw(6)
            << mean[0] << "  std=" << std::setw(6) << stddev[0] << "\n";
}

// 白像素占比，用来量化「二值化 / 边缘」的结果
double white_ratio(const cv::Mat &m) {
  return 100.0 * cv::countNonZero(m) / static_cast<double>(m.total());
}

// 白像素绝对数量，用来量化形态学「到底去掉了多少噪点」
long white_px(const cv::Mat &m) {
  return static_cast<long>(cv::countNonZero(m));
}

// 与参考图的平均绝对差：数值越大说明平滑越强（看 std 变化看不出噪声时用这个）
void mad_line(const std::string &name, const cv::Mat &ref, const cv::Mat &m) {
  cv::Mat diff;
  cv::absdiff(ref, m, diff);
  std::cout << "    " << std::left << std::setw(24) << name << std::right
            << " mean|diff|=" << std::fixed << std::setprecision(2)
            << std::setw(5) << cv::mean(diff)[0] << "\n";
}

// 图像的平均灰度（梯度图用它才有意义）
double mean_gray(const cv::Mat &m) { return cv::mean(m)[0]; }

// 计时器，ms() 返回构造到现在经过的毫秒数
class Timer {
public:
  Timer() : t0_(cv::getTickCount()) {}

  double ms() const {
    return (cv::getTickCount() - t0_) * 1000.0 / cv::getTickFrequency();
  }

private:
  int64 t0_;
};

} // namespace

int main(int argc, char **argv) {
  // -------------------------------------------------------------------------
  // ① imread：读图。注意 OpenCV 默认读成 BGR，不是 RGB
  // -------------------------------------------------------------------------
  section(1, "读图与 cv::Mat 元信息");

  const std::string path = (argc > 1) ? argv[1] : "demo/bule_armoe.jpg";

  Timer t_all;
  cv::Mat img = cv::imread(path);
  if (img.empty()) { // imread 失败不报错，只返回空 Mat，必须自己判断
    std::cerr << "读不到图片：" << path << "\n";
    return 1;
  }

  std::cout << "  路径       " << path << "\n"
            << "  rows×cols  " << img.rows << " × " << img.cols
            << "（行=高，列=宽）\n"
            << "  通道数     " << img.channels() << "\n"
            << "  类型       " << type_name(img.type()) << "\n"
            << "  每行字节   " << img.step[0] << "（连续内存："
            << (img.isContinuous() ? "是" : "否") << "）\n";

  // -------------------------------------------------------------------------
  // ② cvtColor：彩色 → 灰度，后续处理快约 3 倍
  // -------------------------------------------------------------------------
  section(2, "灰度化 cvtColor");

  Timer t_gray;
  cv::Mat gray;
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  std::cout << "  cvtColor(BGR2GRAY) 耗时 " << std::fixed
            << std::setprecision(2) << t_gray.ms() << " ms\n";
  stat_line("gray", gray);

  // -------------------------------------------------------------------------
  // ③ 滤波：都用来「去噪」，区别在于**保不保边**
  // -------------------------------------------------------------------------
  section(3, "滤波去噪 blur / GaussianBlur / medianBlur / bilateralFilter");

  Timer t_filter;
  cv::Mat f_blur, f_gauss, f_median, f_bilateral;
  cv::blur(gray, f_blur, {5, 5});             // 均值：最快，边缘一起糊
  cv::GaussianBlur(gray, f_gauss, {5, 5}, 0); // 高斯：最常用，噪声越远权重越小
  cv::medianBlur(gray, f_median, 5);          // 中值：专杀椒盐噪声，不怎么糊边
  cv::bilateralFilter(gray, f_bilateral, 9, 75, 75); // 双边：保边去噪，慢
  std::cout << "  四种滤波合计 " << std::fixed << std::setprecision(2)
            << t_filter.ms()
            << " ms。mean|diff| = 与原图的平均差别，越大说明抹得越狠：\n";
  mad_line("blur 5x5", gray, f_blur);
  mad_line("GaussianBlur 5x5", gray, f_gauss);
  mad_line("medianBlur 5", gray, f_median);
  mad_line("bilateralFilter", gray, f_bilateral);

  // -------------------------------------------------------------------------
  // ④ 二值化：把灰度图变成黑白，是自瞄里「找灯条」的第一步
  // -------------------------------------------------------------------------
  section(4, "二值化 threshold / THRESH_OTSU / adaptiveThreshold");

  cv::Mat b_fixed, b_otsu, b_adapt_mean, b_adapt_gauss;
  cv::threshold(gray, b_fixed, 150, 255,
                cv::THRESH_BINARY); // 阈值 150 是手填的，换光照就废
  const double otsu_thr = cv::threshold(
      gray, b_otsu, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); // 自动找阈值
  cv::adaptiveThreshold(gray, b_adapt_mean, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                        cv::THRESH_BINARY, 31, 5); // 局部邻域均值
  cv::adaptiveThreshold(gray, b_adapt_gauss, 255,
                        cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 31,
                        5); // 局部高斯加权

  std::cout << "  OTSU 自动算出的阈值 = " << static_cast<int>(otsu_thr)
            << "（本次固定阈值用的是 150）\n";
  std::cout << "  白像素占比：固定 " << white_ratio(b_fixed) << "%   OTSU "
            << white_ratio(b_otsu) << "%   自适应均值 "
            << white_ratio(b_adapt_mean) << "%   自适应高斯 "
            << white_ratio(b_adapt_gauss) << "%\n";

  // -------------------------------------------------------------------------
  // ⑤ 颜色分割：BGR 里红色是 (0,0,255)、蓝色是 (255,0,0)
  //    想按「颜色」筛东西，一律转 HSV 再用 inRange
  // -------------------------------------------------------------------------
  section(5, "颜色分割 cvtColor(BGR2HSV) + inRange");

  cv::Mat hsv;
  cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

  cv::Mat red1, red2, red_mask, blue_mask;
  // 注意：inRange 的上下界是 InputArray，花括号不能自动推成 Scalar，要写全
  // cv::Scalar
  cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255),
              red1); // 红跨 0 度，分两段
  cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), red2);
  cv::bitwise_or(red1, red2, red_mask);
  cv::inRange(hsv, cv::Scalar(100, 100, 100), cv::Scalar(124, 255, 255),
              blue_mask);

  cv::Mat red_only, blue_only;
  cv::bitwise_and(img, img, red_only, red_mask); // 用掩码把原图抠出来
  cv::bitwise_and(img, img, blue_only, blue_mask);

  std::cout << "  red 掩码白像素 " << white_ratio(red_mask)
            << "%   blue 掩码白像素 " << white_ratio(blue_mask) << "%\n";

  // -------------------------------------------------------------------------
  // ⑥ 形态学：在二值图上「修形状」
  // -------------------------------------------------------------------------
  section(6, "形态学 morphologyEx：开 / 闭 / 梯度 / 顶帽");

  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});
  cv::Mat m_open, m_close, m_grad, m_tophat;
  cv::morphologyEx(b_otsu, m_open, cv::MORPH_OPEN,
                   kernel); // 先腐蚀后膨胀：去小噪点
  cv::morphologyEx(b_otsu, m_close, cv::MORPH_CLOSE,
                   kernel); // 先膨胀后腐蚀：补空洞
  cv::morphologyEx(b_otsu, m_grad, cv::MORPH_GRADIENT,
                   kernel); // 膨胀-腐蚀：拿轮廓
  cv::morphologyEx(b_otsu, m_tophat, cv::MORPH_TOPHAT,
                   kernel); // 原图-开运算：拿小亮块

  std::cout << "  白像素个数：OTSU " << white_px(b_otsu) << " → 开运算 "
            << white_px(m_open) << "（少了 "
            << (white_px(b_otsu) - white_px(m_open))
            << " 个孤立噪点） → 闭运算 " << white_px(m_close) << "（多了 "
            << (white_px(m_close) - white_px(b_otsu)) << " 个补上的空洞）\n";

  // -------------------------------------------------------------------------
  // ⑦ 边缘检测：Sobel（方向）/ Laplacian（二阶）/ Canny（最常用）
  // -------------------------------------------------------------------------
  section(7, "边缘检测 Sobel / Laplacian / Canny");

  cv::Mat sx, sy, sxy, lap, canny_lo, canny_hi;
  cv::Sobel(gray, sx, CV_16S, 1, 0, 3); // 横向差分 → 竖边缘
  cv::convertScaleAbs(sx, sx);          // 16S 有正有负，取绝对值才是能显示的图
  cv::Sobel(gray, sy, CV_16S, 0, 1, 3); // 纵向差分 → 横边缘
  cv::convertScaleAbs(sy, sy);
  cv::addWeighted(sx, 0.5, sy, 0.5, 0, sxy); // 两个方向合起来看梯度幅值
  cv::Laplacian(gray, lap, CV_16S, 3);
  cv::convertScaleAbs(lap, lap);
  cv::Canny(gray, canny_lo, 50,
            150); // 双阈值：小的是「起步价」，大的是「确认线」
  cv::Canny(gray, canny_hi, 100, 200); // 阈值调高 → 边缘变少但更干净

  std::cout << "  平均梯度强度：Sobel-|dx| " << std::fixed
            << std::setprecision(1) << mean_gray(sx) << "  Sobel-|dy| "
            << mean_gray(sy) << "  Laplacian " << mean_gray(lap) << "\n";
  std::cout << "  二值边缘（Canny）像素占比：阈值(50,150) → "
            << white_ratio(canny_lo) << "%   阈值(100,200) → "
            << white_ratio(canny_hi) << "%\n";

  // -------------------------------------------------------------------------
  // ⑧ 轮廓：findContours 拿「点集」，再按面积 / 长宽比筛
  //    ⚠ findContours 会修改输入图，养成先 clone 的习惯
  // -------------------------------------------------------------------------
  section(8, "轮廓 findContours + 面积/外接矩形筛选");

  cv::Mat bin = m_open.clone();
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 按面积从大到小排序
  std::sort(contours.begin(), contours.end(), [](const auto &a, const auto &b) {
    return cv::contourArea(a) > cv::contourArea(b);
  });

  const double min_area =
      img.total() * 0.002; // 面积阈值也写成「相对整图」的比例
  int kept = 0;
  cv::Mat contour_vis = img.clone();
  for (const auto &c : contours) {
    const double area = cv::contourArea(c);
    if (area < min_area)
      continue;
    ++kept;

    const cv::Rect box = cv::boundingRect(c);         // 正矩形：快
    const cv::RotatedRect rrect = cv::minAreaRect(c); // 旋转矩形：贴合斜灯条
    const cv::Point2f center = rrect.center;
    const double w = rrect.size.width, h = rrect.size.height;
    if (kept <= 5) { // 只打印最大的 5 个，看参数够不够
      std::cout << "    #" << kept << " 面积=" << static_cast<int>(area)
                << " 正矩形=" << box.width << "×" << box.height
                << " 旋转矩形=" << static_cast<int>(w) << "×"
                << static_cast<int>(h) << " 角度=" << std::fixed
                << std::setprecision(1) << rrect.angle
                << "° 长宽比=" << std::setprecision(2)
                << std::max(w, h) / std::max(std::min(w, h), 1.0) << "\n";
    }

    cv::drawContours(contour_vis, std::vector<std::vector<cv::Point>>{c}, -1,
                     {0, 255, 0}, 2);
    cv::rectangle(contour_vis, box, {255, 160, 0}, 1);
    cv::circle(contour_vis, center, 4, {0, 0, 255}, cv::FILLED);
  }
  std::cout << "  findContours 共 " << contours.size() << " 个外轮廓，面积 ≥ "
            << static_cast<int>(min_area) << "px 的留下 " << kept << " 个\n";
  std::cout
      << "  提示：面积阈值写成 img.total() 的比例，换相机分辨率不用改参数\n";

  // -------------------------------------------------------------------------
  // ⑨ 几何变换：改变「图的大小 / 角度 / 视角」
  // -------------------------------------------------------------------------
  section(9, "几何变换 resize / 旋转 / 透视");

  cv::Mat half;
  cv::resize(img, half, {}, 0.5, 0.5,
             cv::INTER_AREA); // 缩小用 INTER_AREA 才不会出摩尔纹

  // 旋转：先要一个 2×3 的变换矩阵，再 warpAffine
  const cv::Point2f center(img.cols / 2.0f, img.rows / 2.0f);
  const cv::Mat rot_mat =
      cv::getRotationMatrix2D(center, 15.0, 1.0); // 逆时针 15°，不缩放
  cv::Mat rotated;
  cv::warpAffine(img, rotated, rot_mat, img.size());

  // 透视：把任意四边形「拉正」成矩形（装甲板正畸、棋盘格矫正都用它）
  const float w = static_cast<float>(img.cols),
              h = static_cast<float>(img.rows);
  const std::vector<cv::Point2f> src_quad = {{w * 0.30f, h * 0.25f},
                                             {w * 0.70f, h * 0.30f},
                                             {w * 0.75f, h * 0.75f},
                                             {w * 0.25f, h * 0.70f}};
  const std::vector<cv::Point2f> dst_quad = {
      {0, 0}, {400, 0}, {400, 300}, {0, 300}};
  const cv::Mat persp_mat = cv::getPerspectiveTransform(src_quad, dst_quad);
  cv::Mat warped;
  cv::warpPerspective(img, warped, persp_mat, {400, 300});

  std::cout << "  resize 0.5× → " << half.cols << "×" << half.rows << "（原图 "
            << img.cols << "×" << img.rows << "）\n";
  std::cout << "  旋转 15°：getRotationMatrix2D 给一个 2×3 矩阵，warpAffine "
               "的第 4 个参数决定输出尺寸\n";
  std::cout << "  透视变换：把源图里那个四边形拉正成 400×300 的正视图（第 ⑩ "
               "步把它画出来了）\n";

  // -------------------------------------------------------------------------
  // ⑩ 绘制：调试图一定要画出来才看得懂
  // -------------------------------------------------------------------------
  section(10, "绘制 line / rectangle / circle / putText");

  cv::Mat canvas = img.clone();
  cv::rectangle(canvas, cv::Rect(60, 60, 300, 200), {0, 255, 0}, 3); // 绿框
  cv::circle(canvas, {600, 300}, 40, {0, 0, 255}, cv::FILLED);       // 红实心圆
  cv::line(canvas, {100, 400}, {700, 500}, {255, 0, 0}, 3);          // 蓝线
  cv::arrowedLine(canvas, {900, 300}, {1100, 200}, {0, 255, 255}, 3); // 黄箭头
  cv::polylines(
      canvas,
      std::vector<std::vector<cv::Point>>{
          {cv::Point(w * 0.30f, h * 0.25f), cv::Point(w * 0.70f, h * 0.30f),
           cv::Point(w * 0.75f, h * 0.75f), cv::Point(w * 0.25f, h * 0.70f)}},
      true, {255, 0, 255}, 3); // 洋红四边形：第 ⑨ 步透视用的源区域
  cv::putText(canvas, "line / rect / circle / putText", {60, img.rows - 60},
              cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 255}, 2);
  std::cout << "  文字和线宽建议随图尺寸缩放，否则换分辨率就看不见了\n";

  // -------------------------------------------------------------------------
  // 存三张拼图 + 开窗显示
  // -------------------------------------------------------------------------
  const cv::Mat sheet1 = grid({
      {"orig(gray)", gray},
      {"blur 5x5", f_blur},
      {"GaussianBlur 5x5", f_gauss},
      {"medianBlur 5", f_median},
      {"bilateralFilter", f_bilateral},
      {"threshold 150", b_fixed},
      {"OTSU " + std::to_string(static_cast<int>(otsu_thr)), b_otsu},
      {"adaptive MEAN 31", b_adapt_mean},
      {"adaptive GAUSS 31", b_adapt_gauss},
  });

  const cv::Mat sheet2 = grid({
      {"OTSU", b_otsu},
      {"MORPH_OPEN", m_open},
      {"MORPH_CLOSE", m_close},
      {"MORPH_GRADIENT", m_grad},
      {"MORPH_TOPHAT", m_tophat},
      {"Sobel |dx|", sx},
      {"Sobel |dy|", sy},
      {"Sobel mix", sxy},
      {"Laplacian", lap},
  });

  const cv::Mat sheet3 = grid({
      {"red mask", red_mask},
      {"red only", red_only},
      {"blue mask", blue_mask},
      {"blue only", blue_only},
      {"contours", contour_vis},
      {"resize 0.5x", half},
      {"rotate 15deg", rotated},
      {"perspective", warped},
      {"draw", canvas},
  });

  if (!cv::imwrite("opencv_basics_1_filter.png", sheet1) ||
      !cv::imwrite("opencv_basics_2_edge.png", sheet2) ||
      !cv::imwrite("opencv_basics_3_misc.png", sheet3)) {
    std::cerr << "拼图保存失败（检查当前目录是否可写）\n";
  }

  std::cout << "\n总耗时 " << std::fixed << std::setprecision(2) << t_all.ms()
            << " ms\n"
            << "已保存 opencv_basics_1_filter.png / opencv_basics_2_edge.png / "
               "opencv_basics_3_misc.png\n";

  if (std::getenv("NO_WINDOW") == nullptr) {
    cv::imshow("1 filter + threshold", sheet1);
    cv::imshow("2 morphology + edge", sheet2);
    cv::imshow("3 color + contours + warp", sheet3);
    std::cout << "按任意键退出…\n";
    cv::waitKey(0); // 0 = 一直等按键
  }
  return 0;
}
