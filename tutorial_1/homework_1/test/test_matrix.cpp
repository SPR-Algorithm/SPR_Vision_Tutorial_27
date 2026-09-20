// ============================================================================
// 第一阶段作业 · 加分项（矩阵运算）· 自动化测试（GoogleTest）
// 对应作业：tutorial_1/homework_1.md §4
//
// 这套测试只依赖 include/matrix.hpp（它自己会带上 vector.hpp），所以可以
// 「直接套在学员的实现上跑」：把学员的 matrix.hpp / matrix.cpp 放到同样的路径，
// 编译即可。
//
// 三类用例：
//   1. 确定性用例 —— 手算结果，把「对不对」钉死
//   2. 随机数据 · 性质测试 —— 不断言具体数值，而断言必须成立的代数性质
//      （A·A⁻¹ = I、det(AB) = det A · det B、(AB)ᵀ = BᵀAᵀ、乘法结合律……），
//      硬编码答案、按测试用例特判都过不了
//   3. 边界检测 —— 空矩阵 / 非矩形 / 维度不匹配 / 奇异矩阵 / 下标越界 / NaN·inf
// ============================================================================

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "matrix.hpp"

using rm::Matrix;
using rm::Vector;

namespace {

// 普通量的容差
constexpr double kEps = 1e-9;

// 矩阵运算的容差：行列式是余子式递归展开、求逆又要过一遍 det，
// 误差会被放大，比单个标量运算松一档
constexpr double kMatEps = 1e-7;

// 随机用例的规模：维度 1~4、每组 500 次。
// 矩阵是 O(n³) 且行列式走递归，维度再大跑测试就慢了
constexpr int kRandomCases = 500;

// 固定种子的随机数：用例可复现，出问题能原样重跑
std::mt19937 &rng() {
  static std::mt19937 gen(20270101u);
  return gen;
}

double randUniform(double lo, double hi) {
  return std::uniform_real_distribution<double>(lo, hi)(rng());
}

std::size_t randDim(std::size_t lo = 1, std::size_t hi = 4) {
  return lo + rng()() % (hi - lo + 1);
}

Matrix randMatrix(std::size_t n, std::size_t m, double lo = -10.0,
                  double hi = 10.0) {
  Matrix mat(n, Vector(m));
  for (auto &row : mat) {
    for (double &x : row) {
      x = randUniform(lo, hi);
    }
  }
  return mat;
}

// 构造一个「几乎肯定可逆」的随机方阵：
// 随机矩阵行列式为 0 的概率是 0，但小维度下有极小概率抽到接近奇异的，
// 所以显式判一下，真抽到了就重摇
Matrix randInvertible(std::size_t n) {
  for (int attempt = 0; attempt < 200; ++attempt) {
    Matrix mat = randMatrix(n, n);
    if (std::abs(rm::determinant(mat)) > 1e-2) {
      return mat;
    }
  }
  Matrix diag(n, Vector(n, 0.0));
  for (std::size_t i = 0; i < n; ++i) {
    diag[i][i] = 1.0 + static_cast<double>(i);
  }
  return diag;
}

Matrix identity(std::size_t n) {
  Matrix out(n, Vector(n, 0.0));
  for (std::size_t i = 0; i < n; ++i) {
    out[i][i] = 1.0;
  }
  return out;
}

std::string toStr(const Matrix &mat) {
  std::ostringstream os;
  os << '[';
  for (std::size_t i = 0; i < mat.size(); ++i) {
    os << (i == 0 ? "" : ", ") << '[';
    for (std::size_t j = 0; j < mat[i].size(); ++j) {
      os << (j == 0 ? "" : ", ") << mat[i][j];
    }
    os << ']';
  }
  os << ']';
  return os.str();
}

bool allFinite(const Matrix &mat) {
  for (const auto &row : mat) {
    for (const double x : row) {
      if (!std::isfinite(x)) {
        return false;
      }
    }
  }
  return true;
}

// i > j 的位置（对角线以下）是否全为 0
bool isUpperTriangular(const Matrix &mat) {
  for (std::size_t i = 0; i < mat.size(); ++i) {
    for (std::size_t j = 0; j < i && j < mat[i].size(); ++j) {
      if (mat[i][j] != 0.0) {
        return false;
      }
    }
  }
  return true;
}

// 逐元素比较，失败时把两个矩阵都打出来
::testing::AssertionResult matrixNear(const Matrix &a, const Matrix &b,
                                      double eps) {
  if (a.size() != b.size()) {
    return ::testing::AssertionFailure()
           << "行数不同：\n  a = " << toStr(a) << "\n  b = " << toStr(b);
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].size() != b[i].size()) {
      return ::testing::AssertionFailure()
             << "第 " << i << " 行列数不同：\n  a = " << toStr(a)
             << "\n  b = " << toStr(b);
    }
    for (std::size_t j = 0; j < a[i].size(); ++j) {
      if (std::abs(a[i][j] - b[i][j]) > eps) {
        return ::testing::AssertionFailure()
               << "第 (" << i << ", " << j << ") 项不同：" << a[i][j] << " vs "
               << b[i][j] << "\n  a = " << toStr(a) << "\n  b = " << toStr(b);
      }
    }
  }
  return ::testing::AssertionSuccess();
}

// 相对误差比较，用来断言 det(AB) == det(A)·det(B) 这类量级会很大的等式
void expectNearRel(double actual, double expected, double rel,
                   const char *what) {
  EXPECT_NEAR(actual, expected, rel * std::max(1.0, std::abs(expected)))
      << what;
}

// 临时把 std::cout 接到一个 stringstream 上，用来测 show()
class StdoutCapture {
public:
  StdoutCapture() {
    std::cout.flush();
    old_ = std::cout.rdbuf(buf_.rdbuf());
  }
  ~StdoutCapture() { std::cout.rdbuf(old_); }

  StdoutCapture(const StdoutCapture &) = delete;
  StdoutCapture &operator=(const StdoutCapture &) = delete;

  std::string str() {
    std::cout.flush();
    return buf_.str();
  }

private:
  std::ostringstream buf_;
  std::streambuf *old_ = nullptr;
};

} // namespace

// ===========================================================================
// 1. 构造与打印
// ===========================================================================

TEST(Hw1MatrixBasic, ZerosHasRightShapeAndValues) {
  const Matrix m = rm::zeros(2, 3);
  ASSERT_EQ(m.size(), 2u);
  EXPECT_EQ(m[0].size(), 3u);
  for (const auto &row : m) {
    for (const double x : row) {
      EXPECT_DOUBLE_EQ(x, 0.0);
    }
  }
}

TEST(Hw1MatrixBasic, OnesHasRightShapeAndValues) {
  const Matrix m = rm::ones(3, 2);
  ASSERT_EQ(m.size(), 3u);
  EXPECT_EQ(m[0].size(), 2u);
  for (const auto &row : m) {
    for (const double x : row) {
      EXPECT_DOUBLE_EQ(x, 1.0);
    }
  }
}

TEST(Hw1MatrixBasic, RandomStaysInRange) {
  for (int k = 0; k < 200; ++k) {
    const std::size_t n = randDim();
    const std::size_t m = randDim();
    const double lo = randUniform(-50.0, 0.0);
    const double hi = lo + randUniform(0.0, 50.0);

    const Matrix mat = rm::random(n, m, lo, hi);
    ASSERT_EQ(mat.size(), n);
    ASSERT_EQ(mat[0].size(), m);
    for (const auto &row : mat) {
      for (const double x : row) {
        EXPECT_GE(x, lo);
        EXPECT_LE(x, hi);
        EXPECT_TRUE(std::isfinite(x));
      }
    }
  }
}

TEST(Hw1MatrixBasic, RandomWithMinGreaterThanMaxThrows) {
  EXPECT_THROW(rm::random(2, 2, 5.0, 1.0), std::logic_error);
}

TEST(Hw1MatrixBasic, ShowUsesThreeDecimals) {
  const Matrix m{{1.0, -2.5}, {3.14159, 10.0}};

  StdoutCapture cap;
  rm::show(m);
  const std::string out = cap.str();

  EXPECT_NE(out.find("1.000"), std::string::npos) << out;
  EXPECT_NE(out.find("-2.500"), std::string::npos) << out;
  EXPECT_NE(out.find("3.142"), std::string::npos) << out; // 四舍五入到 3 位
  EXPECT_NE(out.find("10.000"), std::string::npos) << out;
  // 不该出现 4 位以上小数
  EXPECT_EQ(out.find("3.14159"), std::string::npos) << out;
  // 两行
  EXPECT_EQ(std::count(out.begin(), out.end(), '\n'), 2) << out;
}

// show 改的是 std::cout 的格式状态，如果实现里直接 std::cout << std::fixed，
// 后面所有输出都会被带上「固定 3 位小数」，这是很典型的副作用 bug
TEST(Hw1MatrixBasic, ShowDoesNotCorruptCoutState) {
  StdoutCapture cap;
  rm::show(Matrix{{1.0}});
  std::cout << 1.0 / 3.0;
  const std::string out = cap.str();

  // 默认是 6 位有效数字；被污染的话这里会是 "0.333"
  EXPECT_NE(out.find("0.333333"), std::string::npos)
      << "show() 污染了 std::cout 的格式状态：" << out;
}

TEST(Hw1MatrixBasic, ShowHandlesEmptyMatrix) {
  StdoutCapture cap;
  rm::show(Matrix{});
  EXPECT_TRUE(cap.str().empty());
}

// ===========================================================================
// 2. 逐元素运算
// ===========================================================================

TEST(Hw1MatrixElementwise, ScalarMultiply) {
  const Matrix m{{1.0, -2.0}, {3.0, 0.5}};
  EXPECT_TRUE(
      matrixNear(rm::multiply(m, 2.0), Matrix{{2.0, -4.0}, {6.0, 1.0}}, kEps));
}

TEST(Hw1MatrixElementwise, ScalarMultiplyByZero) {
  const Matrix m = randMatrix(3, 3);
  EXPECT_TRUE(matrixNear(rm::multiply(m, 0.0), rm::zeros(3, 3), kEps));
}

TEST(Hw1MatrixElementwise, SumWithConstant) {
  const Matrix m{{1.0, 2.0}, {3.0, 4.0}};
  EXPECT_TRUE(
      matrixNear(rm::sum(m, 0.5), Matrix{{1.5, 2.5}, {3.5, 4.5}}, kEps));
}

TEST(Hw1MatrixElementwise, MatrixAddition) {
  const Matrix a{{1.0, 2.0}, {3.0, 4.0}};
  const Matrix b{{10.0, 20.0}, {30.0, 40.0}};
  EXPECT_TRUE(
      matrixNear(rm::sum(a, b), Matrix{{11.0, 22.0}, {33.0, 44.0}}, kEps));
}

TEST(Hw1MatrixElementwise, MatrixAdditionIsCommutative) {
  for (int k = 0; k < kRandomCases; ++k) {
    const std::size_t n = randDim();
    const std::size_t m = randDim();
    const Matrix a = randMatrix(n, m);
    const Matrix b = randMatrix(n, m);
    EXPECT_TRUE(matrixNear(rm::sum(a, b), rm::sum(b, a), kEps));
  }
}

TEST(Hw1MatrixElementwise, MatrixAdditionMismatchThrows) {
  EXPECT_THROW(rm::sum(rm::zeros(2, 3), rm::zeros(3, 2)), std::logic_error);
  EXPECT_THROW(rm::sum(rm::zeros(2, 3), rm::zeros(1, 3)), std::logic_error);
}

TEST(Hw1MatrixElementwise, TransposeKnown) {
  const Matrix m{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
  const Matrix expected{{1.0, 4.0}, {2.0, 5.0}, {3.0, 6.0}};
  EXPECT_TRUE(matrixNear(rm::transpose(m), expected, kEps));
}

TEST(Hw1MatrixElementwise, TransposeTwiceIsIdentity) {
  for (int k = 0; k < kRandomCases; ++k) {
    const Matrix m = randMatrix(randDim(), randDim());
    EXPECT_TRUE(matrixNear(rm::transpose(rm::transpose(m)), m, kEps));
  }
}

TEST(Hw1MatrixElementwise, TransposeIsInvolutiveOnShape) {
  const Matrix m = randMatrix(3, 5);
  const Matrix t = rm::transpose(m);
  EXPECT_EQ(t.size(), 5u);
  EXPECT_EQ(t[0].size(), 3u);
}

TEST(Hw1MatrixElementwise, TransposeOfEmptyIsEmpty) {
  EXPECT_TRUE(rm::transpose(Matrix{}).empty());
}

// ===========================================================================
// 3. 矩阵乘法
// ===========================================================================

TEST(Hw1MatrixMultiply, KnownNonSquareProduct) {
  const Matrix a{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};      // 2x3
  const Matrix b{{7.0, 8.0}, {9.0, 10.0}, {11.0, 12.0}}; // 3x2
  const Matrix expected{{58.0, 64.0}, {139.0, 154.0}};   // 2x2
  EXPECT_TRUE(matrixNear(rm::multiply(a, b), expected, kEps));
}

TEST(Hw1MatrixMultiply, IdentityIsNeutral) {
  for (int k = 0; k < kRandomCases; ++k) {
    const std::size_t n = randDim();
    const std::size_t m = randDim();
    const Matrix a = randMatrix(n, m);
    EXPECT_TRUE(matrixNear(rm::multiply(a, identity(m)), a, kMatEps));
    EXPECT_TRUE(matrixNear(rm::multiply(identity(n), a), a, kMatEps));
  }
}

TEST(Hw1MatrixMultiply, IsAssociative) {
  for (int k = 0; k < kRandomCases; ++k) {
    const std::size_t n = randDim(1, 4);
    const std::size_t p = randDim(1, 4);
    const std::size_t q = randDim(1, 4);
    const std::size_t r = randDim(1, 4);

    const Matrix a = randMatrix(n, p);
    const Matrix b = randMatrix(p, q);
    const Matrix c = randMatrix(q, r);

    EXPECT_TRUE(matrixNear(rm::multiply(rm::multiply(a, b), c),
                           rm::multiply(a, rm::multiply(b, c)), kMatEps));
  }
}

TEST(Hw1MatrixMultiply, DistributesOverAddition) {
  for (int k = 0; k < kRandomCases; ++k) {
    const std::size_t n = randDim(1, 4);
    const std::size_t p = randDim(1, 4);
    const std::size_t q = randDim(1, 4);

    const Matrix a = randMatrix(n, p);
    const Matrix b = randMatrix(p, q);
    const Matrix c = randMatrix(p, q);

    EXPECT_TRUE(matrixNear(rm::multiply(a, rm::sum(b, c)),
                           rm::sum(rm::multiply(a, b), rm::multiply(a, c)),
                           kMatEps));
  }
}

TEST(Hw1MatrixMultiply, TransposeOfProduct) {
  for (int k = 0; k < kRandomCases; ++k) {
    const std::size_t n = randDim(1, 4);
    const std::size_t p = randDim(1, 4);
    const std::size_t q = randDim(1, 4);

    const Matrix a = randMatrix(n, p);
    const Matrix b = randMatrix(p, q);

    // (AB)ᵀ = BᵀAᵀ
    EXPECT_TRUE(matrixNear(rm::transpose(rm::multiply(a, b)),
                           rm::multiply(rm::transpose(b), rm::transpose(a)),
                           kMatEps));
  }
}

TEST(Hw1MatrixMultiply, DimensionMismatchThrows) {
  EXPECT_THROW(rm::multiply(rm::zeros(2, 3), rm::zeros(2, 3)),
               std::logic_error);
  EXPECT_THROW(rm::multiply(rm::zeros(2, 3), rm::zeros(4, 5)),
               std::logic_error);
}

TEST(Hw1MatrixMultiply, ByZeroMatrixIsZero) {
  const Matrix a = randMatrix(3, 2);
  EXPECT_TRUE(
      matrixNear(rm::multiply(a, rm::zeros(2, 4)), rm::zeros(3, 4), kEps));
}

// ===========================================================================
// 4. 行列式
// ===========================================================================

TEST(Hw1MatrixDeterminant, EmptyIsOne) {
  // 作业里明确约定：空矩阵的行列式为 1
  EXPECT_DOUBLE_EQ(rm::determinant(Matrix{}), 1.0);
}

TEST(Hw1MatrixDeterminant, Known1x1And2x2) {
  EXPECT_DOUBLE_EQ(rm::determinant(Matrix{{7.0}}), 7.0);
  EXPECT_DOUBLE_EQ(rm::determinant(Matrix{{4.0, 7.0}, {2.0, 6.0}}), 10.0);
}

TEST(Hw1MatrixDeterminant, Known3x3) {
  const Matrix m{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 10.0}};
  EXPECT_NEAR(rm::determinant(m), -3.0, kEps);
}

TEST(Hw1MatrixDeterminant, IdentityIsOne) {
  for (std::size_t n = 1; n <= 5; ++n) {
    EXPECT_NEAR(rm::determinant(identity(n)), 1.0, kEps) << "n = " << n;
  }
}

TEST(Hw1MatrixDeterminant, TriangularIsProductOfDiagonal) {
  for (int k = 0; k < 100; ++k) {
    const std::size_t n = randDim(1, 5);
    Matrix m = randMatrix(n, n, 0.5, 3.0); // 对角元取正，避免符号干扰
    double expected = 1.0;
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = 0; j < i; ++j) {
        m[i][j] = 0.0; // 下三角清零
      }
      expected *= m[i][i];
    }
    expectNearRel(rm::determinant(m), expected, 1e-9,
                  "上三角行列式应为对角元之积");
  }
}

TEST(Hw1MatrixDeterminant, IsMultiplicative) {
  for (int k = 0; k < kRandomCases; ++k) {
    const std::size_t n = randDim(1, 4);
    const Matrix a = randMatrix(n, n);
    const Matrix b = randMatrix(n, n);

    // det(AB) = det(A) · det(B)
    expectNearRel(rm::determinant(rm::multiply(a, b)),
                  rm::determinant(a) * rm::determinant(b), 1e-6,
                  "det(AB) ≠ det(A)·det(B)");
  }
}

TEST(Hw1MatrixDeterminant, RowSwapFlipsSign) {
  for (int k = 0; k < kRandomCases; ++k) {
    const std::size_t n = randDim(2, 4);
    const Matrix a = randMatrix(n, n);
    // 交换两行，行列式变号
    expectNearRel(rm::determinant(rm::ero_swap(a, 0, 1)), -rm::determinant(a),
                  1e-6, "交换两行行列式应变号");
  }
}

TEST(Hw1MatrixDeterminant, DuplicateRowIsZero) {
  for (int k = 0; k < 100; ++k) {
    const std::size_t n = randDim(2, 4);
    Matrix m = randMatrix(n, n);
    m[1] = m[0]; // 两行相同 → 奇异
    EXPECT_NEAR(rm::determinant(m), 0.0, 1e-9);
  }
}

TEST(Hw1MatrixDeterminant, NonSquareThrows) {
  EXPECT_THROW(rm::determinant(rm::zeros(2, 3)), std::logic_error);
  EXPECT_THROW(rm::determinant(rm::zeros(3, 2)), std::logic_error);
}

// ===========================================================================
// 5. 逆矩阵
// ===========================================================================

TEST(Hw1MatrixInverse, Known2x2) {
  const Matrix m{{4.0, 7.0}, {2.0, 6.0}}; // det = 10
  const Matrix expected{{0.6, -0.7}, {-0.2, 0.4}};
  EXPECT_TRUE(matrixNear(rm::inverse(m), expected, kEps));
}

TEST(Hw1MatrixInverse, TimesOriginalIsIdentity) {
  for (int k = 0; k < kRandomCases; ++k) {
    const std::size_t n = randDim(1, 4);
    const Matrix a = randInvertible(n);
    const Matrix inv = rm::inverse(a);

    EXPECT_TRUE(matrixNear(rm::multiply(a, inv), identity(n), kMatEps));
    EXPECT_TRUE(matrixNear(rm::multiply(inv, a), identity(n), kMatEps));
  }
}

TEST(Hw1MatrixInverse, InverseOfIdentityIsIdentity) {
  for (std::size_t n = 1; n <= 5; ++n) {
    EXPECT_TRUE(matrixNear(rm::inverse(identity(n)), identity(n), kEps))
        << "n = " << n;
  }
}

TEST(Hw1MatrixInverse, InverseOfInverseIsOriginal) {
  for (int k = 0; k < 100; ++k) {
    const Matrix a = randInvertible(randDim(1, 4));
    EXPECT_TRUE(matrixNear(rm::inverse(rm::inverse(a)), a, kMatEps));
  }
}

TEST(Hw1MatrixInverse, SingularThrows) {
  const Matrix singular{{1.0, 2.0}, {2.0, 4.0}}; // 第二行是第一行的 2 倍
  EXPECT_THROW(rm::inverse(singular), std::logic_error);

  // 含全零行的必然奇异
  Matrix with_zero_row = randMatrix(3, 3);
  with_zero_row[1] = Vector(3, 0.0);
  EXPECT_THROW(rm::inverse(with_zero_row), std::logic_error);
}

TEST(Hw1MatrixInverse, NonSquareThrows) {
  EXPECT_THROW(rm::inverse(rm::zeros(2, 3)), std::logic_error);
}

TEST(Hw1MatrixInverse, EmptyThrows) {
  EXPECT_THROW(rm::inverse(Matrix{}), std::logic_error);
}

// ===========================================================================
// 6. minor（子矩阵）
// ===========================================================================

TEST(Hw1MatrixMinor, DropsRowAndColumn) {
  const Matrix m{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
  EXPECT_TRUE(
      matrixNear(rm::minor(m, 1, 1), Matrix{{1.0, 3.0}, {7.0, 9.0}}, kEps));
  EXPECT_TRUE(
      matrixNear(rm::minor(m, 0, 2), Matrix{{4.0, 5.0}, {7.0, 8.0}}, kEps));
  EXPECT_TRUE(
      matrixNear(rm::minor(m, 2, 0), Matrix{{2.0, 3.0}, {5.0, 6.0}}, kEps));
}

TEST(Hw1MatrixMinor, OfOneByOneIsEmpty) {
  EXPECT_TRUE(rm::minor(Matrix{{5.0}}, 0, 0).empty());
}

TEST(Hw1MatrixMinor, NonSquareStillWorks) {
  const Matrix m{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}}; // 2x3
  const Matrix sub = rm::minor(m, 0, 1); // 去掉第 0 行、第 1 列 → 1x2
  ASSERT_EQ(sub.size(), 1u);
  EXPECT_EQ(sub[0].size(), 2u);
  EXPECT_TRUE(matrixNear(sub, Matrix{{4.0, 6.0}}, kEps));
}

TEST(Hw1MatrixMinor, OutOfRangeThrows) {
  const Matrix m = randMatrix(2, 3);
  EXPECT_THROW(rm::minor(m, 2, 0), std::logic_error); // 行越界
  EXPECT_THROW(rm::minor(m, 0, 3), std::logic_error); // 列越界
}

// ===========================================================================
// 7. concatenate（拼接）
// ===========================================================================

TEST(Hw1MatrixConcatenate, Vertical) {
  const Matrix a{{1.0, 2.0}};
  const Matrix b{{3.0, 4.0}};
  EXPECT_TRUE(matrixNear(rm::concatenate(a, b, 0),
                         Matrix{{1.0, 2.0}, {3.0, 4.0}}, kEps));
}

TEST(Hw1MatrixConcatenate, Horizontal) {
  const Matrix a{{1.0, 2.0}};
  const Matrix b{{3.0, 4.0}};
  EXPECT_TRUE(
      matrixNear(rm::concatenate(a, b, 1), Matrix{{1.0, 2.0, 3.0, 4.0}}, kEps));
}

TEST(Hw1MatrixConcatenate, DefaultAxisIsVertical) {
  const Matrix a{{1.0}, {2.0}};
  const Matrix b{{3.0}, {4.0}};
  EXPECT_TRUE(matrixNear(rm::concatenate(a, b),
                         Matrix{{1.0}, {2.0}, {3.0}, {4.0}}, kEps));
}

TEST(Hw1MatrixConcatenate, ShapesAddUp) {
  const Matrix a = randMatrix(2, 3);
  const Matrix b = randMatrix(4, 3);
  const Matrix c = randMatrix(2, 5);

  EXPECT_EQ(rm::concatenate(a, b, 0).size(), 6u);
  EXPECT_EQ(rm::concatenate(a, c, 1).size(), 2u);
  EXPECT_EQ(rm::concatenate(a, c, 1)[0].size(), 8u);
}

TEST(Hw1MatrixConcatenate, Axis0ColumnMismatchThrows) {
  EXPECT_THROW(rm::concatenate(rm::zeros(2, 3), rm::zeros(2, 4), 0),
               std::logic_error);
}

TEST(Hw1MatrixConcatenate, Axis1RowMismatchThrows) {
  EXPECT_THROW(rm::concatenate(rm::zeros(2, 3), rm::zeros(3, 3), 1),
               std::logic_error);
}

TEST(Hw1MatrixConcatenate, BadAxisThrows) {
  EXPECT_THROW(rm::concatenate(rm::zeros(2, 2), rm::zeros(2, 2), 2),
               std::logic_error);
  EXPECT_THROW(rm::concatenate(rm::zeros(2, 2), rm::zeros(2, 2), -1),
               std::logic_error);
}

// ===========================================================================
// 8. 初等行变换
// ===========================================================================

TEST(Hw1MatrixEro, Swap) {
  const Matrix m{{1.0, 2.0}, {3.0, 4.0}};
  EXPECT_TRUE(
      matrixNear(rm::ero_swap(m, 0, 1), Matrix{{3.0, 4.0}, {1.0, 2.0}}, kEps));
  // 自己和自己换是恒等
  EXPECT_TRUE(matrixNear(rm::ero_swap(m, 0, 0), m, kEps));
}

TEST(Hw1MatrixEro, Multiply) {
  const Matrix m{{1.0, 2.0}, {3.0, 4.0}};
  EXPECT_TRUE(matrixNear(rm::ero_multiply(m, 1, 2.0),
                         Matrix{{1.0, 2.0}, {6.0, 8.0}}, kEps));
  EXPECT_TRUE(matrixNear(rm::ero_multiply(m, 0, 0.0),
                         Matrix{{0.0, 0.0}, {3.0, 4.0}}, kEps));
}

TEST(Hw1MatrixEro, Sum) {
  const Matrix m{{1.0, 2.0}, {3.0, 4.0}};
  // 第 1 行 += 10 × 第 0 行
  EXPECT_TRUE(matrixNear(rm::ero_sum(m, 0, 10.0, 1),
                         Matrix{{1.0, 2.0}, {13.0, 24.0}}, kEps));
  // c == 0 时不改变任何东西
  EXPECT_TRUE(matrixNear(rm::ero_sum(m, 0, 0.0, 1), m, kEps));
}

TEST(Hw1MatrixEro, DoesNotModifyInput) {
  const Matrix m = randMatrix(3, 3);
  const Matrix before = m;
  (void)rm::ero_swap(m, 0, 2);
  (void)rm::ero_multiply(m, 0, 3.0);
  (void)rm::ero_sum(m, 0, 3.0, 2);
  EXPECT_TRUE(matrixNear(m, before, kEps)) << "ero_* 不应该修改入参";
}

TEST(Hw1MatrixEro, EroSumIsReversible) {
  // 把 r1 的 c 倍加到 r2 上，再把 r1 的 -c 倍加到 r2 上，应该回到原样
  for (int k = 0; k < 200; ++k) {
    const std::size_t n = randDim(2, 5);
    const Matrix m = randMatrix(n, n);
    const double c = randUniform(-5.0, 5.0);
    const Matrix once = rm::ero_sum(m, 0, c, 1);
    const Matrix twice = rm::ero_sum(once, 0, -c, 1);
    EXPECT_TRUE(matrixNear(twice, m, kMatEps));
  }
}

TEST(Hw1MatrixEro, OutOfRangeThrows) {
  const Matrix m = randMatrix(2, 3);
  EXPECT_THROW(rm::ero_swap(m, 0, 2), std::logic_error);
  EXPECT_THROW(rm::ero_multiply(m, 2, 1.0), std::logic_error);
  EXPECT_THROW(rm::ero_sum(m, 2, 1.0, 0), std::logic_error);
  EXPECT_THROW(rm::ero_sum(m, 0, 1.0, 2), std::logic_error);
}

// ===========================================================================
// 9. upper_triangular（上三角化）
// ===========================================================================

TEST(Hw1MatrixUpperTriangular, KnownCase) {
  const Matrix m{{1.0, 2.0}, {3.0, 4.0}};
  const Matrix out = rm::upper_triangular(m);

  // 部分主元：第 0 列里 |3| > |1|，所以会先换行
  EXPECT_TRUE(matrixNear(out, Matrix{{3.0, 4.0}, {0.0, 2.0 / 3.0}}, kMatEps))
      << "实际 = " << toStr(out);
}

TEST(Hw1MatrixUpperTriangular, ZerosBelowDiagonal) {
  for (int k = 0; k < kRandomCases; ++k) {
    const Matrix m = randMatrix(randDim(1, 5), randDim(1, 5));
    const Matrix out = rm::upper_triangular(m);
    EXPECT_TRUE(isUpperTriangular(out)) << "下三角没有清零：" << toStr(out);
  }
}

TEST(Hw1MatrixUpperTriangular, AlreadyTriangularIsUnchanged) {
  for (int k = 0; k < 200; ++k) {
    const std::size_t n = randDim(1, 5);
    Matrix m = randMatrix(n, n, 0.5, 3.0); // 对角元非零
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = 0; j < i; ++j) {
        m[i][j] = 0.0;
      }
    }
    EXPECT_TRUE(matrixNear(rm::upper_triangular(m), m, kMatEps));
  }
}

TEST(Hw1MatrixUpperTriangular, KeepsShape) {
  const Matrix m = randMatrix(3, 5);
  const Matrix out = rm::upper_triangular(m);
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0].size(), 5u);
}

TEST(Hw1MatrixUpperTriangular, PreservesAbsoluteDeterminant) {
  for (int k = 0; k < kRandomCases; ++k) {
    const std::size_t n = randDim(1, 4);
    const Matrix m = randMatrix(n, n);
    // 高斯消元只做「行交换（变号）+ 行倍加（不变）」，所以 |det| 保持不变
    expectNearRel(std::abs(rm::determinant(rm::upper_triangular(m))),
                  std::abs(rm::determinant(m)), 1e-6,
                  "|det(upper_triangular(A))| ≠ |det(A)|");
  }
}

TEST(Hw1MatrixUpperTriangular, ZeroMatrixStaysZero) {
  EXPECT_TRUE(
      matrixNear(rm::upper_triangular(rm::zeros(3, 3)), rm::zeros(3, 3), kEps));
  EXPECT_TRUE(rm::upper_triangular(Matrix{}).empty());
}

TEST(Hw1MatrixUpperTriangular, RankDeficientColumnDoesNotThrow) {
  // 第一列全 0，没有主元可用；实现不能在这里死循环或崩掉，要跳到下一列
  Matrix m = randMatrix(3, 3);
  for (std::size_t i = 0; i < 3; ++i) {
    m[i][0] = 0.0;
  }
  const Matrix out = rm::upper_triangular(m);
  EXPECT_TRUE(isUpperTriangular(out));
  EXPECT_TRUE(allFinite(out));
}

// ===========================================================================
// 10. 边界与非法输入
// ===========================================================================

TEST(Hw1MatrixEdge, NonRectangularRejected) {
  const Matrix ragged{{1.0, 2.0}, {3.0}}; // 第 1 行少一列

  EXPECT_THROW(rm::transpose(ragged), std::logic_error);
  EXPECT_THROW(rm::determinant(ragged), std::logic_error);
  EXPECT_THROW(rm::inverse(ragged), std::logic_error);
  EXPECT_THROW(rm::upper_triangular(ragged), std::logic_error);
  EXPECT_THROW(rm::concatenate(ragged, rm::zeros(2, 2), 0), std::logic_error);
  EXPECT_THROW(rm::sum(ragged, rm::zeros(2, 2)), std::logic_error);
  EXPECT_THROW(rm::multiply(ragged, rm::zeros(2, 2)), std::logic_error);
}

TEST(Hw1MatrixEdge, EmptyMatrixBehavesConsistently) {
  const Matrix empty{};

  EXPECT_TRUE(rm::transpose(empty).empty());
  EXPECT_TRUE(rm::multiply(empty, 2.0).empty());
  EXPECT_TRUE(rm::sum(empty, 1.0).empty());
  EXPECT_TRUE(rm::upper_triangular(empty).empty());
  EXPECT_DOUBLE_EQ(rm::determinant(empty), 1.0);
  EXPECT_TRUE(rm::concatenate(empty, empty, 0).empty());
  EXPECT_TRUE(rm::concatenate(empty, rm::zeros(2, 2), 0).size() == 2u);
  EXPECT_TRUE(rm::concatenate(rm::zeros(2, 3), empty, 0).size() == 2u);
}

TEST(Hw1MatrixEdge, ZeroSizedDimension) {
  // 2 行 0 列：不是方阵，但仍应能被安全地转置 / 拼接
  const Matrix m = rm::zeros(2, 0);
  ASSERT_EQ(m.size(), 2u);
  EXPECT_EQ(m[0].size(), 0u);

  const Matrix t = rm::transpose(m);
  ASSERT_EQ(t.size(), 0u);
}

TEST(Hw1MatrixEdge, NoNaNOrInfAnywhere) {
  for (int k = 0; k < 200; ++k) {
    const std::size_t n = randDim(1, 5);
    const Matrix m = randMatrix(n, n);

    EXPECT_TRUE(allFinite(rm::multiply(m, 2.5))) << toStr(m);
    EXPECT_TRUE(allFinite(rm::sum(m, 3.0))) << toStr(m);
    EXPECT_TRUE(allFinite(rm::transpose(m))) << toStr(m);
    EXPECT_TRUE(allFinite(rm::multiply(m, rm::transpose(m)))) << toStr(m);
    EXPECT_TRUE(allFinite(rm::upper_triangular(m))) << toStr(m);

    const double det = rm::determinant(m);
    EXPECT_TRUE(std::isfinite(det)) << toStr(m);

    // 奇异矩阵要抛异常，不能算出 inf / NaN 还返回
    if (std::abs(det) > 1e-2) {
      EXPECT_TRUE(allFinite(rm::inverse(m))) << toStr(m);
    } else {
      EXPECT_THROW(rm::inverse(m), std::logic_error) << toStr(m);
    }
  }
}

TEST(Hw1MatrixEdge, DeterminantOfSymmetricMatrixIsFinite) {
  // A · Aᵀ 永远对称半正定，det 一定非负
  for (int k = 0; k < 200; ++k) {
    const std::size_t n = randDim(1, 4);
    const Matrix a = randMatrix(n, n);
    const Matrix sym = rm::multiply(a, rm::transpose(a));
    const double det = rm::determinant(sym);
    EXPECT_TRUE(std::isfinite(det));
    EXPECT_GE(det, -1e-6) << "A·Aᵀ 的行列式不应该为负：" << toStr(sym);
  }
}
