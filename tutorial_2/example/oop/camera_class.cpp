// ============================================================================
// 知识点 1 · C++ 面向对象 —— 基于相机类的开发
// 对应教案：一、C++ 面向对象（1.5 基于相机类的开发）
//
// 上课时执行：
//   cd example/oop
//   g++ -std=c++17 -Wall -Wextra camera_class.cpp -o camera && ./camera
//
// 这个例子刻意不依赖 OpenCV，用最朴素的 Frame 代替 cv::Mat，
// 目的是让学员把注意力集中在「抽象基类 + 多态」这件事本身。
// 到了第二节，Frame 会被换成 cv::Mat，但类的结构一模一样。
//
// 演示要点（讲师按这个顺序讲）：
//   1. 抽象基类 Camera：定义「相机应该会做什么」，不能实例化
//   2. 两个派生类：各自实现 open/read/close/name
//   3. 工厂函数：把配置里的字符串翻译成具体对象
//   4. 使用方 runOnce()：只认 Camera&，完全不关心具体型号 ← 这是重点
//   5. 加一个 ReplayCamera：换硬件/做回归测试时算法代码一行不用改
// ============================================================================

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// 图像帧：先用一个最简单的结构体代替 cv::Mat
// ---------------------------------------------------------------------------
struct Frame {
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> gray; // 灰度数据，长度 = width * height
};

// ---------------------------------------------------------------------------
// 抽象基类：只定义「相机应该会做什么」，不关心具体是哪一种相机
// ---------------------------------------------------------------------------
class Camera {
public:
  virtual ~Camera() = default; // ⚠ 基类析构必须是虚的

  virtual bool open() = 0; // 纯虚函数 = 0 → Camera 不能被实例化
  virtual bool read(Frame &out) = 0;
  virtual void close() = 0;
  virtual std::string name() const = 0;
};

// ---------------------------------------------------------------------------
// 派生类 1：USB 相机
// ---------------------------------------------------------------------------
class UsbCamera : public Camera {
public:
  explicit UsbCamera(int device_index) : index_(device_index) {}

  ~UsbCamera() override { close(); } // 析构里保证资源被释放（RAII）

  bool open() override {
    opened_ = true;
    std::cout << "  [UsbCamera /dev/video" << index_
              << "] open() 成功（模拟）\n";
    return true;
  }

  bool read(Frame &out) override {
    if (!opened_) {
      return false;
    }
    out.width = 640;
    out.height = 480;
    out.gray.assign(static_cast<std::size_t>(out.width) * out.height,
                    static_cast<std::uint8_t>(frame_ * 8)); // 模拟画面变化
    ++frame_;
    return true;
  }

  void close() override {
    if (opened_) {
      std::cout << "  [UsbCamera /dev/video" << index_ << "] close()\n";
    }
    opened_ = false;
  }

  std::string name() const override {
    return "UsbCamera(/dev/video" + std::to_string(index_) + ")";
  }

private:
  int index_ = 0;
  bool opened_ = false;
  int frame_ = 0;
};

// ---------------------------------------------------------------------------
// 派生类 2：工业相机（走厂商 SDK，和 USB 相机的取图方式完全不同）
// ---------------------------------------------------------------------------
class IndustrialCamera : public Camera {
public:
  explicit IndustrialCamera(int serial) : serial_(serial) {}

  ~IndustrialCamera() override { close(); }

  bool open() override {
    opened_ = true;
    std::cout << "  [IndustrialCamera SN" << serial_
              << "] open() 成功（模拟）\n";
    return true;
  }

  bool read(Frame &out) override {
    if (!opened_) {
      return false;
    }
    out.width = 1280; // 工业相机分辨率更高
    out.height = 1024;
    out.gray.assign(static_cast<std::size_t>(out.width) * out.height,
                    static_cast<std::uint8_t>(frame_ * 4));
    ++frame_;
    return true;
  }

  void close() override {
    if (opened_) {
      std::cout << "  [IndustrialCamera SN" << serial_ << "] close()\n";
    }
    opened_ = false;
  }

  std::string name() const override {
    return "IndustrialCamera(SN" + std::to_string(serial_) + ")";
  }

private:
  int serial_ = 0;
  bool opened_ = false;
  int frame_ = 0;
};

// ---------------------------------------------------------------------------
// 派生类 3：回放相机 —— 读录像文件，用于离线回归测试
// 加这个类的时候，算法层（runOnce）一行都不用改，这就是抽象的价值
// ---------------------------------------------------------------------------
class ReplayCamera : public Camera {
public:
  explicit ReplayCamera(const std::string &path) : path_(path) {}

  ~ReplayCamera() override { close(); }

  bool open() override {
    opened_ = true;
    std::cout << "  [ReplayCamera " << path_ << "] open() 成功（模拟）\n";
    return true;
  }

  bool read(Frame &out) override {
    if (!opened_ || frame_ >= 3) { // 录像只有 3 帧，放完就结束
      return false;
    }
    out.width = 640;
    out.height = 480;
    out.gray.assign(static_cast<std::size_t>(out.width) * out.height,
                    static_cast<std::uint8_t>(frame_ * 40));
    ++frame_;
    return true;
  }

  void close() override {
    if (opened_) {
      std::cout << "  [ReplayCamera " << path_ << "] close()\n";
    }
    opened_ = false;
  }

  std::string name() const override { return "ReplayCamera(" + path_ + ")"; }

private:
  std::string path_;
  bool opened_ = false;
  int frame_ = 0;
};

// ---------------------------------------------------------------------------
// 工厂：把「配置里的字符串」翻译成「具体对象」
// 以后新增一种相机，只需要在这里加一行，调用方完全不用动
// ---------------------------------------------------------------------------
std::unique_ptr<Camera> makeCamera(const std::string &type, int id) {
  if (type == "usb") {
    return std::make_unique<UsbCamera>(id);
  }
  if (type == "industrial") {
    return std::make_unique<IndustrialCamera>(id);
  }
  if (type == "replay") {
    return std::make_unique<ReplayCamera>("demo.bag");
  }
  std::cerr << "未知相机类型：" << type << '\n';
  return nullptr;
}

// ---------------------------------------------------------------------------
// 算法层：这里就是「识别装甲板」的位置。
// 它只认 Camera&，所以换成哪种相机都能跑，换硬件时这里一行不用改。
// ---------------------------------------------------------------------------
void runOnce(Camera &cam) {
  Frame frame;
  if (!cam.read(frame)) { // 多态：运行期决定调用哪个 read()
    std::cout << "  " << cam.name() << " 没有取到图\n";
    return;
  }
  std::cout << "  " << cam.name() << " 出图：" << frame.width << "x"
            << frame.height << "，灰度数据 " << frame.gray.size() << " 字节"
            << "，首像素=" << static_cast<int>(frame.gray.front()) << '\n';
}

int main() {
  std::cout << "=== 1. 两种相机用同一段代码取图 ===\n";
  {
    std::vector<std::unique_ptr<Camera>> cameras;
    cameras.push_back(makeCamera("usb", 0));
    cameras.push_back(makeCamera("industrial", 20270101));

    for (auto &cam : cameras) { // unique_ptr 独占所有权，不需要手动 delete
      if (cam == nullptr || !cam->open()) {
        continue;
      }
      runOnce(*cam); // 同一个函数，两种相机
      cam->close();
    }
  }

  std::cout << "\n=== 2. 换成回放相机做回归测试，算法代码一行没改 ===\n";
  {
    auto cam = makeCamera("replay", 0);
    if (cam->open()) {
      for (int i = 0; i < 4; ++i) { // 录像放完会返回 false
        runOnce(*cam);
      }
      cam->close();
    }
  }

  std::cout << "\n=== 3. 直接用引用调用（多态的最小验证）===\n";
  {
    UsbCamera usb(1);
    runOnce(usb); // 没 open，read() 应该返回 false
    usb.open();
    runOnce(usb);
  }

  std::cout
      << "\n提醒：程序结束时会自动析构，注意看第 3 组的 close() 是谁打的\n";
  return 0;
}
