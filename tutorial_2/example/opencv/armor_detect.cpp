// ============================================================================
// 知识点 1 · OpenCV —— 用传统方法识别装甲板
// 对应教案：一、OpenCV（1.2 环境搭建与最小 CMake 工程 / 1.4 用传统方法识别装甲板）
//
//   cd example/opencv
//   cmake -S . -B build && cmake --build build
//   ./build/armor_detect                       # 默认读 demo/bule_armoe.jpg
//   ./build/armor_detect my_armor.png          # 也可以喂自己的图
//   DEBUG_PAIRS=1 ./build/armor_detect         # 打印所有候选配对及其得分
//   NO_WINDOW=1  ./build/armor_detect          # 不开窗口，只存图片
//
//   灰度 → 二值化(OTSU) → 形态学开运算 → findContours
//        → 灯条筛选（长宽比 / 长短粗细）
//        → 灯条配对（间距 / 上下端对齐 / 高度相近 / 宽高比）
//        → 打分取最优 → 输出装甲板【四个角点】
// 输出的这四个点（左上 / 右上 / 右下 / 左下）就是本节的最终成果，
//
// ============================================================================

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#if __has_include(<opencv2/geometry/2d.hpp>)
#include <opencv2/geometry/2d.hpp> // OpenCV 5 起把 minAreaRect 从 imgproc 挪到了这里
#endif

namespace {

// ---------------------------------------------------------------------------
// 数据结构
// ---------------------------------------------------------------------------

// 一根候选灯条（用「上端点 + 下端点」描述，比直接用 RotatedRect 好算）
struct LightBar {
  cv::RotatedRect rect;
  cv::Point2f top;    // 上边中点
  cv::Point2f bottom; // 下边中点
  float height = 0.0f;
  float width = 0.0f;
};

// 识别结果
struct Armor {
  bool found = false;
  LightBar left;
  LightBar right;
  float aspect = 0.0f; // 宽高比 = 两灯条间距 / 平均灯条高度
  float score = 0.0f;  // 越接近真实装甲板比例，得分越高
};

struct DetectResult {
  Armor armor;
  std::vector<LightBar> light_bars; // 所有通过筛选的候选灯条（画出来看更直观）
  cv::Mat gray;                     // ① 灰度图
  cv::Mat binary;                   // ② 二值化（OTSU）后
  cv::Mat opened;                   // ③ 滤波（形态学开运算）后
};

// 真实装甲板的「灯条中心距 / 灯条高度」大约在 2.2 ~ 2.8 之间
constexpr float kIdealAspect = 2.5f;

// 打分时给「灯条相对大小」的权重。
// 只比宽高比不够用：背景里的噪点也可能凑出 2.4 左右的宽高比，
// 但真正的装甲板灯条一定占画面很大比例，所以大小要参与打分。
constexpr float kSizeWeight = 3.0f;

// ---------------------------------------------------------------------------
// 第 1 步：把旋转矩形归一化成「上端点 + 下端点」
// ---------------------------------------------------------------------------
LightBar makeLightBar(const cv::RotatedRect &rect) {
  cv::Point2f pts[4];
  rect.points(pts);

  // 四个顶点按 y 排序：y 最小的两个构成上边，y 最大的两个构成下边
  std::vector<cv::Point2f> p(pts, pts + 4);
  std::sort(p.begin(), p.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
    return a.y < b.y;
  });

  LightBar bar;
  bar.rect = rect;
  bar.top = (p[0] + p[1]) * 0.5f;
  bar.bottom = (p[2] + p[3]) * 0.5f;
  bar.height = static_cast<float>(cv::norm(bar.top - bar.bottom));
  bar.width = std::min(rect.size.width, rect.size.height);
  return bar;
}

// ---------------------------------------------------------------------------
// 第 2 步：灯条筛选
// ---------------------------------------------------------------------------
bool isLightBar(const LightBar &bar, const cv::Size &img) {
  // 所有约束都写成「相对整图尺寸」，换分辨率不用改参数
  if (bar.height < img.height * 0.05f)
    return false; // 太短：背景噪点
  if (bar.height > img.height * 0.60f)
    return false; // 太长：灯管 / 背景边缘
  if (bar.width < 2.0f)
    return false; // 太细：噪点
  if (bar.width > img.width * 0.08f)
    return false; // 太粗：装甲板上的白色数字等大色块

  const float ratio = bar.height / std::max(bar.width, 1.0f);
  return ratio >= 1.5f && ratio <= 15.0f; // 不够细长就不是灯条
}

// ---------------------------------------------------------------------------
// 第 3 步：二值图 → 轮廓 → 候选灯条
// ---------------------------------------------------------------------------
std::vector<LightBar> findLightBars(const cv::Mat &binary,
                                    const cv::Size &img_size) {
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  // findContours 会修改输入图，所以传 clone()
  cv::findContours(binary.clone(), contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  std::vector<LightBar> bars;
  for (const auto &contour : contours) {
    if (contour.size() < 4)
      continue;
    const LightBar bar = makeLightBar(cv::minAreaRect(contour));
    if (isLightBar(bar, img_size)) {
      bars.push_back(bar);
    }
  }
  return bars;
}

// ---------------------------------------------------------------------------
// 第 4 步：灯条配对
// ---------------------------------------------------------------------------
std::optional<Armor> tryMatch(const LightBar &a, const LightBar &b,
                              const cv::Size &img) {
  // 先约定左右顺序
  const LightBar &L = (a.top.x < b.top.x) ? a : b;
  const LightBar &R = (a.top.x < b.top.x) ? b : a;

  // ① 两灯条间距要在合理范围内
  //    上限放宽到 0.95 倍图宽：近距离拍摄时装甲板几乎占满画面，
  //    「像不像一块装甲板」主要交给下面第 ⑤ 条的宽高比来把关
  const float gap = static_cast<float>(cv::norm(L.top - R.top));
  if (gap < 10.0f || gap > img.width * 0.95f)
    return std::nullopt;

  // ② 上端点应该大致在一条水平线上
  if (std::abs(L.top.y - R.top.y) > L.height * 0.6f)
    return std::nullopt;

  // ③ 下端点同理
  if (std::abs(L.bottom.y - R.bottom.y) > L.height * 0.6f)
    return std::nullopt;

  // ④ 两根灯条高度要相近（同一个装甲板上的灯条是一样长的）
  const float height_ratio = L.height / std::max(R.height, 1.0f);
  if (height_ratio < 0.6f || height_ratio > 1.67f)
    return std::nullopt;

  // ⑤ 装甲板的宽高比要合理（这一步能挡掉大部分随机配对）
  const float avg_height = (L.height + R.height) * 0.5f;
  const float aspect = gap / std::max(avg_height, 1.0f);
  if (aspect < 1.0f || aspect > 5.0f)
    return std::nullopt;

  Armor armor;
  armor.found = true;
  armor.left = L;
  armor.right = R;
  armor.aspect = aspect;
  // 得分 = 宽高比接近程度 + 灯条相对大小
  armor.score = -std::abs(aspect - kIdealAspect) +
                kSizeWeight * (avg_height / static_cast<float>(img.height));
  return armor;
}

// ---------------------------------------------------------------------------
// 把上面四步串起来：输入 BGR 原图，输出识别结果
// ---------------------------------------------------------------------------
DetectResult detectArmor(const cv::Mat &bgr) {
  DetectResult result;

  // ① 灰度：3 通道 → 1 通道
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

  // ② 二值化：OTSU 自动找阈值，比写死 127 稳得多
  cv::Mat binary;
  cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  // ③ 形态学开运算：去掉小噪点
  const cv::Mat kernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat opened;
  cv::morphologyEx(binary, opened, cv::MORPH_OPEN, kernel);

  // 三步的中间结果都留在 result 里，后面拼成一张图显示
  result.gray = gray;
  result.binary = binary;
  result.opened = opened;

  // ④⑤ 找轮廓 + 筛灯条
  //  注意：喂进去的是【滤波后】的图，二值化原图只用来显示 ——
  //  开运算抹掉了细碎噪点，轮廓质量比直接用二值图好
  result.light_bars = findLightBars(opened, bgr.size());

  // ⑥ 两两配对，留得分最高的那一组
  float best_score = -1e9f;
  for (std::size_t i = 0; i < result.light_bars.size(); ++i) {
    for (std::size_t j = i + 1; j < result.light_bars.size(); ++j) {
      const auto armor =
          tryMatch(result.light_bars[i], result.light_bars[j], bgr.size());
      if (armor.has_value()) {
        if (std::getenv("DEBUG_PAIRS") != nullptr) {
          std::printf("[候选] 左(%.0f,%.0f)h%.0f 右(%.0f,%.0f)h%.0f → 宽高比 "
                      "%.2f 得分 %.3f\n",
                      armor->left.top.x, armor->left.top.y, armor->left.height,
                      armor->right.top.x, armor->right.top.y,
                      armor->right.height, armor->aspect, armor->score);
        }
        if (armor->score > best_score) {
          best_score = armor->score;
          result.armor = *armor;
        }
      }
    }
  }
  return result;
}

// ---------------------------------------------------------------------------
// 可视化：候选灯条画黄色，最终装甲板画绿色
// ---------------------------------------------------------------------------
void drawResult(cv::Mat &img, const DetectResult &result) {
  for (const auto &bar : result.light_bars) {
    cv::Point2f pts[4];
    bar.rect.points(pts);
    for (int i = 0; i < 4; ++i) {
      cv::line(img, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 255), 1);
    }
  }

  // 文字和线宽随图片尺寸缩放，否则在大图上根本看不清
  const double scale = std::max(0.6, img.cols / 1200.0);
  const int thickness = static_cast<int>(std::max(2.0, scale * 2.0));

  if (!result.armor.found) {
    cv::putText(img, "NO ARMOR", cv::Point(20, static_cast<int>(50 * scale)),
                cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 255),
                thickness);
    return;
  }

  const auto &a = result.armor;
  // 四点顺序固定为：左上 → 右上 → 右下 → 左下
  // 这个顺序就是后面做位姿解算（solvePnP）时要喂进去的顺序
  const std::vector<cv::Point2f> corners = {a.left.top, a.right.top,
                                            a.right.bottom, a.left.bottom};

  // 装甲板外框（绿色）—— 这四个点就是识别的最终输出
  for (int i = 0; i < 4; ++i) {
    cv::line(img, corners[i], corners[(i + 1) % 4], cv::Scalar(0, 255, 0),
             thickness);
  }

  // 四个角点单独标出来，方便课上逐个指认
  static const char *kCornerNames[4] = {"P0", "P1", "P2", "P3"};
  for (int i = 0; i < 4; ++i) {
    cv::circle(img, corners[i], thickness * 2, cv::Scalar(0, 255, 0),
               cv::FILLED);
    cv::putText(img, kCornerNames[i], corners[i] + cv::Point2f(12.0f, -12.0f),
                cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 255, 255),
                thickness);
  }

  // 中心点（红色）
  cv::circle(img, (corners[0] + corners[2]) * 0.5f, thickness * 2,
             cv::Scalar(0, 0, 255), cv::FILLED);

  char text[128];
  std::snprintf(text, sizeof(text), "armor  aspect=%.2f", a.aspect);
  cv::putText(img, text, cv::Point(20, static_cast<int>(50 * scale)),
              cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 255, 0),
              thickness);
}

// ---------------------------------------------------------------------------
// 把「灰度 → 二值化 → 滤波」三张中间图横着拼成一张
// 每格都等比缩放到同样宽度，拼起来才不会一大一小
// 注意：图里写的字只能用 ASCII —— Hershey 字体画不了中文，会变成一串问号
// ---------------------------------------------------------------------------
cv::Mat makeStageCanvas(const DetectResult &result, int tile_width = 480) {
  const cv::Mat stages[3] = {result.gray, result.binary, result.opened};
  const char *titles[3] = {"(1) gray", "(2) threshold (OTSU)",
                           "(3) morphology OPEN"};

  std::vector<cv::Mat> tiles;
  for (int i = 0; i < 3; ++i) {
    if (stages[i].empty())
      continue; // 没跑到这一步就跳过

    // 单通道图（灰度 / 二值）先转成 3 通道，才能和彩色图拼在一起
    cv::Mat tile;
    if (stages[i].channels() == 1) {
      cv::cvtColor(stages[i], tile, cv::COLOR_GRAY2BGR);
    } else {
      tile = stages[i].clone();
    }

    // 统缩放到 tile_width 宽（本来就比它小就不放大，免得糊）
    const double scale = static_cast<double>(tile_width) / tile.cols;
    if (scale < 1.0) {
      cv::resize(tile, tile, cv::Size(), scale, scale, cv::INTER_AREA);
    }

    // 顶部留 28 像素黑边写步骤名，免得和图像内容混在一起看不清
    cv::Mat labeled(tile.rows + 28, tile.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    tile.copyTo(labeled(cv::Rect(0, 28, tile.cols, tile.rows)));
    cv::putText(labeled, titles[i], cv::Point(8, 20), cv::FONT_HERSHEY_SIMPLEX,
                0.6, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    tiles.push_back(labeled);
  }

  if (tiles.empty())
    return cv::Mat();

  cv::Mat canvas;
  cv::hconcat(tiles, canvas);
  return canvas;
}

// ---------------------------------------------------------------------------
// 保存 + 显示中间过程图
// NO_WINDOW=1 时只存图不开窗（服务器 / 容器里没有显示环境）
// ---------------------------------------------------------------------------
void showStages(const cv::Mat &stage_canvas, const cv::Mat &result_canvas) {
  if (stage_canvas.empty())
    return;

  cv::imwrite("debug_stages.png", stage_canvas);
  std::cout << "中间过程图已保存为 debug_stages.png（灰度 / 二值化 / 滤波）\n";

  if (std::getenv("NO_WINDOW") != nullptr)
    return;

  // 服务器 / 容器里没有显示环境，开窗会抛异常，catch 住别让程序直接挂掉
  try {
    cv::imshow("stages: 1 gray | 2 binary | 3 filtered", stage_canvas);
    cv::imshow("armor_detect", result_canvas);
    std::cout << "显示中间过程图与识别结果，按任意键关闭窗口...\n";
    cv::waitKey(0);
    cv::destroyAllWindows();
  } catch (const cv::Exception &e) {
    std::cout
        << "（当前环境开不了窗口，看 debug_stages.png / result.png 即可）\n";
  }
}

} // namespace

// ---------------------------------------------------------------------------
// 单张图片：识别 + 打印 + 存 result.png + 可选开窗
// ---------------------------------------------------------------------------
int runImage(const std::string &source) {
  cv::Mat img = cv::imread(source); // 注意：读失败不报错，只返回空 Mat

  if (img.empty()) {
    std::cerr << "读不到图片：" << source << "\n"
              << "用法：./armor_detect [图片路径]，不带参数默认读 "
                 "demo/bule_armoe.jpg\n";
    return 1;
  }

  std::cout << "已读入 " << source << "（" << img.cols << "x" << img.rows
            << "）\n";

  const DetectResult result = detectArmor(img);

  std::cout << "\n候选灯条 " << result.light_bars.size() << " 根：\n";
  for (std::size_t i = 0; i < result.light_bars.size(); ++i) {
    const auto &bar = result.light_bars[i];
    std::printf("  #%zu 中心(%.0f, %.0f) 高=%.0f 宽=%.0f 长宽比=%.2f\n", i,
                bar.rect.center.x, bar.rect.center.y, bar.height, bar.width,
                bar.height / bar.width);
  }

  cv::Mat canvas = img.clone();
  drawResult(canvas, result);

  if (result.armor.found) {
    const auto &a = result.armor;
    const std::vector<cv::Point2f> corners = {a.left.top, a.right.top,
                                              a.right.bottom, a.left.bottom};
    static const char *kNames[4] = {"左上 P0", "右上 P1", "右下 P2", "左下 P3"};

    std::printf("\n识别到装甲板：\n");
    std::printf("  宽高比   %.2f（理想值 %.1f）\n", a.aspect, kIdealAspect);
    std::printf("  灯条间距 %.0f 像素\n", a.right.top.x - a.left.top.x);
    std::printf("  灯条高度 %.0f / %.0f 像素\n", a.left.height, a.right.height);
    std::printf("  四点坐标（顺序 = 左上 右上 右下 左下）：\n");
    for (int i = 0; i < 4; ++i) {
      std::printf("    %-8s (%7.1f, %7.1f)\n", kNames[i], corners[i].x,
                  corners[i].y);
    }
  } else {
    std::cout << "\n未识别到装甲板\n";
  }

  cv::imwrite("result.png", canvas);
  std::cout << "结果图已保存为 result.png\n";

  // 灰度 / 二值化 / 滤波 三张中间图：保存 + 开窗显示（NO_WINDOW=1 只保存）
  showStages(makeStageCanvas(result), canvas);

  return 0;
}

// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
  // 不带参数时读仓库自带的演示图，保证一定能现场演示
  const std::string source = (argc >= 2) ? argv[1] : "demo/bule_armoe.jpg";

  // 视频部分暂时不做：那涉及整车建模观测与坐标系转换，属于后面的内容。
  // 这一段只做一件事：单张图 → 装甲板四点。
  return runImage(source);
}
