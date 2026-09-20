// ============================================================================
// 第一阶段作业 · 自动化测试（GoogleTest）
// 对应作业：tutorial_1/homework_1.md
//
// 这套测试只依赖 include/rm/vector.hpp 一个头文件，所以可以「直接套在学员的
// 实现上跑」：把学员的 vector.hpp / vector.cpp 放到同样的路径，编译即可。
//
// 三类用例：
//   1. 确定性用例 —— 手算结果，把「对不对」钉死
//   2. 随机数据 · 性质测试 —— 不断言具体数值，而断言必须成立的数学性质，
//      硬编码答案、按测试用例特判都过不了
//   3. 边界检测 —— 零向量 / 维度不一致 / 近零向量 / 空向量 / NaN·inf
// ============================================================================

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
// 推荐头文件名称
#include "vector.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

// 普通量的容差
constexpr double kEps = 1e-9;

// 夹角的容差要放宽到 1e-6：acos 在 ±1 附近极其敏感，
// acos(1 - 1e-16) ≈ 1.4e-8，这不是实现有问题，是函数本身的病态
constexpr double kAngleEps = 1e-6;

constexpr int kRandomCases = 2000;

// 固定种子的随机数：用例可复现，出问题能原样重跑
std::mt19937 &rng() {
  static std::mt19937 gen(20270101u);
  return gen;
}

double randUniform(double lo, double hi) {
  return std::uniform_real_distribution<double>(lo, hi)(rng());
}

std::size_t randDim(std::size_t lo = 1, std::size_t hi = 8) {
  return lo + rng()() % (hi - lo + 1);
}

rm::Vector randVector(std::size_t n, double lo = -100.0, double hi = 100.0) {
  rm::Vector v(n);
  for (double &x : v) {
    x = randUniform(lo, hi);
  }
  return v;
}

std::string toStr(const rm::Vector &v) {
  std::ostringstream os;
  os << '(';
  for (std::size_t i = 0; i < v.size(); ++i) {
    os << (i == 0 ? "" : ", ") << v[i];
  }
  os << ')';
  return os.str();
}

bool allFinite(const rm::Vector &v) {
  for (const double x : v) {
    if (!std::isfinite(x)) {
      return false;
    }
  }
  return true;
}

// a - b，用来单独验证 distance 的定义
rm::Vector diff(const rm::Vector &a, const rm::Vector &b) {
  rm::Vector d(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    d[i] = a[i] - b[i];
  }
  return d;
}

} // namespace

// ===========================================================================
// 1. 确定性用例
// ===========================================================================

TEST(Hw1Vector, KnownValues) {
  const rm::Vector a{3.0, 4.0};
  const rm::Vector b{1.0, 0.0};

  EXPECT_NEAR(rm::length(a), 5.0, kEps);
  EXPECT_NEAR(rm::distance(a, b), std::sqrt(20.0), kEps);
  EXPECT_NEAR(rm::dot(a, b), 3.0, kEps);

  const rm::Vector s = rm::scale(a, 2.0);
  ASSERT_EQ(s.size(), 2u);
  EXPECT_DOUBLE_EQ(s[0], 6.0);
  EXPECT_DOUBLE_EQ(s[1], 8.0);

  const rm::Vector u = rm::normalize(a);
  ASSERT_EQ(u.size(), 2u);
  EXPECT_NEAR(u[0], 0.6, kEps);
  EXPECT_NEAR(u[1], 0.8, kEps);

  // 夹角 = arccos(3/5) ≈ 53.13°
  EXPECT_NEAR(rm::angleBetween(a, b), std::acos(0.6), kEps);
  EXPECT_NEAR(rm::angleBetween(a, b) * 180.0 / kPi, 53.13010235, 1e-6);
}

TEST(Hw1Vector, AngleSpecialCases) {
  const rm::Vector x{2.0, 0.0, 0.0};

  EXPECT_NEAR(rm::angleBetween(x, rm::Vector{0.0, 5.0, 0.0}), kPi / 2,
              kEps) // 垂直
      << "垂直向量夹角应为 90°";
  EXPECT_NEAR(rm::angleBetween(x, rm::Vector{7.0, 0.0, 0.0}), 0.0,
              kAngleEps) // 同向
      << "同向向量夹角应为 0°";
  EXPECT_NEAR(rm::angleBetween(x, rm::Vector{-3.0, 0.0, 0.0}), kPi,
              kAngleEps) // 反向
      << "反向向量夹角应为 180°";
}

TEST(Hw1Vector, WorksForAnyDimension) {
  for (std::size_t n = 1; n <= 16; ++n) {
    const rm::Vector ones(n, 1.0);
    const rm::Vector zeros(n, 0.0);
    const double expected = std::sqrt(static_cast<double>(n));

    EXPECT_NEAR(rm::length(ones), expected, kEps) << "n = " << n;
    EXPECT_NEAR(rm::distance(ones, zeros), expected, kEps) << "n = " << n;
    EXPECT_NEAR(rm::dot(ones, ones), static_cast<double>(n), kEps)
        << "n = " << n;
    EXPECT_NEAR(rm::length(rm::normalize(ones)), 1.0, kEps) << "n = " << n;

    const rm::Vector scaled = rm::scale(ones, 3.0);
    ASSERT_EQ(scaled.size(), n);
    for (const double x : scaled) {
      EXPECT_DOUBLE_EQ(x, 3.0) << "n = " << n;
    }
  }
}

// ===========================================================================
// 2. 随机数据 · 性质测试
// ===========================================================================

TEST(Hw1Random, NormalizeGivesUnitLength) {
  for (int t = 0; t < kRandomCases; ++t) {
    const std::size_t n = randDim();
    const rm::Vector v = randVector(n);
    if (rm::length(v) < 1e-6) {
      continue;
    }

    const rm::Vector u = rm::normalize(v);
    ASSERT_EQ(u.size(), n) << "单位化不该改变维度，v = " << toStr(v);
    EXPECT_NEAR(rm::length(u), 1.0, kEps)
        << "单位向量的模长必须是 1，v = " << toStr(v);
  }
}

TEST(Hw1Random, DistanceIsLengthOfDifference) {
  for (int t = 0; t < kRandomCases; ++t) {
    const std::size_t n = randDim();
    const rm::Vector a = randVector(n);
    const rm::Vector b = randVector(n);

    EXPECT_NEAR(rm::distance(a, b), rm::length(diff(a, b)), kEps)
        << "distance 必须是 |a - b|，a = " << toStr(a) << " b = " << toStr(b);
    EXPECT_NEAR(rm::distance(a, b), rm::distance(b, a), kEps) << "距离应对称";
    EXPECT_DOUBLE_EQ(rm::distance(a, a), 0.0) << "同一点距离应为 0";
  }
}

TEST(Hw1Random, DotProperties) {
  for (int t = 0; t < kRandomCases; ++t) {
    const std::size_t n = randDim();
    const rm::Vector a = randVector(n);
    const rm::Vector b = randVector(n);
    const double k = randUniform(-5.0, 5.0);

    EXPECT_NEAR(rm::dot(a, b), rm::dot(b, a), kEps) << "点乘应对称";
    EXPECT_NEAR(rm::dot(a, rm::scale(b, k)), k * rm::dot(a, b), kEps)
        << "点乘对缩放线性，a = " << toStr(a) << " b = " << toStr(b)
        << " k = " << k;

    // 柯西-施瓦茨：|a·b| ≤ |a|·|b|
    const double lhs = std::abs(rm::dot(a, b));
    const double rhs = rm::length(a) * rm::length(b);
    EXPECT_LE(lhs, rhs * (1.0 + 1e-12))
        << "柯西-施瓦茨不成立，a = " << toStr(a) << " b = " << toStr(b);
  }
}

TEST(Hw1Random, TriangleInequality) {
  for (int t = 0; t < 500; ++t) {
    const std::size_t n = randDim();
    const rm::Vector a = randVector(n);
    const rm::Vector b = randVector(n);
    const rm::Vector c = randVector(n);

    EXPECT_LE(rm::distance(a, c),
              rm::distance(a, b) + rm::distance(b, c) + kEps)
        << "三角不等式不成立，a = " << toStr(a) << " b = " << toStr(b)
        << " c = " << toStr(c);
  }
}

TEST(Hw1Random, AngleProperties) {
  for (int t = 0; t < kRandomCases; ++t) {
    const std::size_t n = randDim();
    const rm::Vector a = randVector(n);
    const rm::Vector b = randVector(n);
    if (rm::length(a) < 1e-6 || rm::length(b) < 1e-6) {
      continue;
    }

    const double ab = rm::angleBetween(a, b);
    EXPECT_NEAR(ab, rm::angleBetween(b, a), kEps) << "夹角应对称";
    EXPECT_GE(ab, 0.0) << "夹角不该为负";
    EXPECT_LE(ab, kPi + kEps) << "夹角不该超过 180°";

    EXPECT_NEAR(rm::angleBetween(a, a), 0.0, kAngleEps) << "和自己夹角应为 0";
    EXPECT_NEAR(rm::angleBetween(a, rm::scale(a, -2.0)), kPi, kAngleEps)
        << "反平行的夹角应为 180°";
  }
}

TEST(Hw1Random, NormalizeIsInvariantUnderPositiveScale) {
  for (int t = 0; t < 1000; ++t) {
    const std::size_t n = randDim();
    const rm::Vector v = randVector(n);
    if (rm::length(v) < 1e-6) {
      continue;
    }

    const double k = randUniform(0.01, 100.0);
    const rm::Vector u1 = rm::normalize(v);
    const rm::Vector u2 = rm::normalize(rm::scale(v, k));

    ASSERT_EQ(u1.size(), u2.size());
    for (std::size_t i = 0; i < u1.size(); ++i) {
      EXPECT_NEAR(u1[i], u2[i], kEps)
          << "正数缩放不该改变方向，v = " << toStr(v) << " k = " << k;
    }
  }
}

TEST(Hw1Random, ScaleProperties) {
  for (int t = 0; t < 1000; ++t) {
    const std::size_t n = randDim();
    const rm::Vector v = randVector(n);

    const rm::Vector by_one = rm::scale(v, 1.0);
    const rm::Vector by_zero = rm::scale(v, 0.0);
    ASSERT_EQ(by_one.size(), n);
    ASSERT_EQ(by_zero.size(), n);

    for (std::size_t i = 0; i < n; ++i) {
      EXPECT_DOUBLE_EQ(by_one[i], v[i]) << "乘 1 应不变";
      EXPECT_DOUBLE_EQ(by_zero[i], 0.0) << "乘 0 应得零向量";
    }

    EXPECT_NEAR(rm::length(rm::scale(v, 2.0)), 2.0 * rm::length(v), kEps);
    EXPECT_NEAR(rm::length(rm::scale(v, -2.0)), 2.0 * rm::length(v), kEps)
        << "模长与符号无关";
  }
}

// ===========================================================================
// 3. 边界检测
// ===========================================================================

TEST(Hw1Edge, NormalizeZeroVectorDoesNotDivideByZero) {
  const std::vector<rm::Vector> zeros = {rm::Vector{0.0, 0.0}, rm::Vector{0.0},
                                         rm::Vector(5, 0.0), rm::Vector{}};

  for (const rm::Vector &z : zeros) {
    const rm::Vector u = rm::normalize(z);

    EXPECT_EQ(u.size(), z.size())
        << "零向量单位化不该改变维度，输入 " << toStr(z);
    for (const double x : u) {
      EXPECT_TRUE(std::isfinite(x)) << "出现了非有限值，输入 " << toStr(z);
      EXPECT_DOUBLE_EQ(x, 0.0) << "零向量单位化应返回全 0（不能除零），输入 "
                               << toStr(z) << " 实际得到 " << toStr(u);
    }
  }
}

TEST(Hw1Edge, AngleWithZeroVectorIsDefined) {
  const rm::Vector z{0.0, 0.0};
  const rm::Vector v{3.0, 4.0};

  EXPECT_TRUE(std::isfinite(rm::angleBetween(z, v)));
  EXPECT_TRUE(std::isfinite(rm::angleBetween(v, z)));
  EXPECT_DOUBLE_EQ(rm::angleBetween(z, v), 0.0) << "零向量的夹角约定为 0";
  EXPECT_DOUBLE_EQ(rm::angleBetween(v, z), 0.0) << "交换参数结论应一致";
  EXPECT_DOUBLE_EQ(rm::angleBetween(z, z), 0.0);
}

TEST(Hw1Edge, DimensionMismatchThrows) {
  const rm::Vector a{1.0, 2.0, 3.0};
  const rm::Vector b{1.0, 2.0};

  EXPECT_THROW(rm::distance(a, b), std::logic_error)
      << "维度不一致应抛 logic_error";
  EXPECT_THROW(rm::dot(a, b), std::logic_error) << "维度不一致应抛 logic_error";
  EXPECT_THROW(rm::angleBetween(a, b), std::logic_error)
      << "维度不一致应抛 logic_error";
  EXPECT_NO_THROW(rm::distance(a, a)) << "同维度不该抛异常";
}

TEST(Hw1Edge, NearZeroVectorIsTreatedAsZero) {
  // 模长 1e-12 远小于容差 1e-9：应被当成零向量处理，而不是拿去当除数
  const rm::Vector tiny{1e-12, 0.0};

  const rm::Vector u = rm::normalize(tiny);
  for (const double x : u) {
    EXPECT_DOUBLE_EQ(x, 0.0) << "近乎零的向量应视为零向量，实际 " << toStr(u);
  }
  EXPECT_DOUBLE_EQ(rm::angleBetween(tiny, rm::Vector{1.0, 0.0}), 0.0)
      << "近乎零的向量夹角应按零向量处理";
}

TEST(Hw1Edge, EmptyVector) {
  const rm::Vector e;

  EXPECT_DOUBLE_EQ(rm::length(e), 0.0);
  EXPECT_DOUBLE_EQ(rm::dot(e, e), 0.0);
  EXPECT_DOUBLE_EQ(rm::distance(e, e), 0.0);
  EXPECT_DOUBLE_EQ(rm::angleBetween(e, e), 0.0);
  EXPECT_TRUE(rm::normalize(e).empty()) << "空向量单位化后仍应为空";
  EXPECT_TRUE(rm::scale(e, 5.0).empty());
}

TEST(Hw1Edge, AcosDoesNotProduceNaN) {
  // 平行 / 反平行最容易让 cos 因为浮点误差越过 ±1，acos 直接返回 NaN
  for (int t = 0; t < kRandomCases; ++t) {
    const std::size_t n = randDim();
    const rm::Vector a = randVector(n);
    if (rm::length(a) < 1e-6) {
      continue;
    }

    const double k = randUniform(0.01, 100.0);
    const double same = rm::angleBetween(a, rm::scale(a, k));
    const double opposite = rm::angleBetween(a, rm::scale(a, -k));

    EXPECT_TRUE(std::isfinite(same))
        << "同向夹角算出了非有限值，a = " << toStr(a);
    EXPECT_TRUE(std::isfinite(opposite))
        << "反向夹角算出了非有限值，a = " << toStr(a);
    EXPECT_NEAR(same, 0.0, kAngleEps) << "a = " << toStr(a);
    EXPECT_NEAR(opposite, kPi, kAngleEps) << "a = " << toStr(a);
  }
}

TEST(Hw1Edge, NoNaNOrInfAnywhere) {
  for (int t = 0; t < kRandomCases; ++t) {
    const std::size_t n = randDim();
    rm::Vector a = randVector(n);
    rm::Vector b = randVector(n);

    if (t % 8 == 0) {
      a = rm::Vector(n, 0.0); // 掺入零向量
    } else if (t % 8 == 1) {
      b = rm::Vector(n, 0.0);
    } else if (t % 8 == 2) {
      a = rm::Vector(n, 1e-12); // 掺入近零向量
    } else if (t % 8 == 3) {
      b = rm::Vector(n, -1e-12);
    }

    EXPECT_TRUE(std::isfinite(rm::length(a))) << "length, a = " << toStr(a);
    EXPECT_TRUE(std::isfinite(rm::distance(a, b)))
        << "distance, a = " << toStr(a) << " b = " << toStr(b);
    EXPECT_TRUE(std::isfinite(rm::dot(a, b)))
        << "dot, a = " << toStr(a) << " b = " << toStr(b);
    EXPECT_TRUE(allFinite(rm::scale(a, 3.0))) << "scale, a = " << toStr(a);
    EXPECT_TRUE(allFinite(rm::normalize(a))) << "normalize, a = " << toStr(a);
    EXPECT_TRUE(std::isfinite(rm::angleBetween(a, b)))
        << "angleBetween, a = " << toStr(a) << " b = " << toStr(b);
  }
}
