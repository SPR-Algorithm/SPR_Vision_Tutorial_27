# 第一阶段作业：二维向量运算库

对应教案：[`tutorial_1.md`](tutorial_1.md)（git 工作流见教案 §2.3）

---

## 1. 题目

写一个简单的**二维向量运算库**：计算二维向量的距离、模长、点乘、缩放，实现单位化与夹角。

**类型定义**

```cpp
#include <vector>

namespace rm {

using Vector = std::vector<double>;                // n 维向量，二维就是 size() == 2
using Matrix = std::vector<std::vector<double>>;   // 矩阵

}  // namespace rm
```

**接口格式**（参数与返回值按这个来）

| 接口           | 签名                                                    |
| -------------- | ------------------------------------------------------- |
| `length`       | `double length(const Vector& v)`                        |
| `distance`     | `double distance(const Vector& a, const Vector& b)`     |
| `dot`          | `double dot(const Vector& a, const Vector& b)`          |
| `scale`        | `Vector scale(const Vector& v, double k)`               |
| `normalize`    | `Vector normalize(const Vector& v)`                     |
| `angleBetween` | `double angleBetween(const Vector& a, const Vector& b)` |

**边界检测**：每个接口先判断输入是否合法再计算；零向量不能除零；参与运算的向量维度必须一致；全程不允许出现 `NaN` / `inf`。

## 2. 要求

1. **git**：完成 `clone`、`commit`、`push`；提交信息符合 Conventional Commits。
2. **交付**：只交库本身（头文件 + 实现 + `CMakeLists.txt`），不用写 `main()`。
3. **CMake**：自己写 `CMakeLists.txt`，用 `add_library` 把运算部分做成静态库，并通过 `target_include_directories` 把 `include/` 暴露给使用者。
4. **规范**：头文件用 `#pragma once`；声明与实现分离。

## 3. 验收清单

**交付物**

- [ ] 作业提交在**自己的分支** `feat/<你的名字>` 上，**没有**直接提交到 `main`
- [ ] push 之前已经从 `main` 同步过最新内容（`git rebase main`）
- [ ] 提交记录需要符合 Conventional Commits（`feat` / `fix` / `build` / `docs` …）
- [ ] 仓库里**没有**提交 `build/` 或编译产物（`.o` / `.a` / `.so`），有 `.gitignore`

## 4. 加分项：矩阵运算（选做）

> 在 6 个向量接口全部通过的基础上额外实现。**不影响第一阶段是否通过**，只在总评里加分。

**类型**：复用第 1 节定义的 `rm::Matrix`

```cpp
using Matrix = std::vector<std::vector<double>>;
```

**接口**

| 接口                 | 签名                                                                   |
| -------------------- | ---------------------------------------------------------------------- |
| `zeros`              | `Matrix zeros(size_t n, size_t m)`                                     |
| `ones`               | `Matrix ones(size_t n, size_t m)`                                      |
| `random`             | `Matrix random(size_t n, size_t m, double min, double max)`            |
| `show`               | `void show(const Matrix& matrix)`                                      |
| `multiply`（数乘）   | `Matrix multiply(const Matrix& matrix, double c)`                      |
| `multiply`（矩阵乘） | `Matrix multiply(const Matrix& m1, const Matrix& m2)`                  |
| `sum`（加常数）      | `Matrix sum(const Matrix& matrix, double c)`                           |
| `sum`（矩阵加）      | `Matrix sum(const Matrix& m1, const Matrix& m2)`                       |
| `transpose`          | `Matrix transpose(const Matrix& matrix)`                               |
| `minor`              | `Matrix minor(const Matrix& matrix, size_t n, size_t m)`               |
| `determinant`        | `double determinant(const Matrix& matrix)`                             |
| `inverse`            | `Matrix inverse(const Matrix& matrix)`                                 |
| `concatenate`        | `Matrix concatenate(const Matrix& m1, const Matrix& m2, int axis = 0)` |
| `ero_swap`           | `Matrix ero_swap(const Matrix& matrix, size_t r1, size_t r2)`          |
| `ero_multiply`       | `Matrix ero_multiply(const Matrix& matrix, size_t r, double c)`        |
| `ero_sum`            | `Matrix ero_sum(const Matrix& matrix, size_t r1, double c, size_t r2)` |
| `upper_triangular`   | `Matrix upper_triangular(const Matrix& matrix)`                        |

**约束**

- 行列式**用余子式展开**
- 随机数用 `<random>`
- `show` 用 `<iomanip>`，每个元素固定 **3 位小数**、对齐输出
- `concatenate` 的 `axis=0` 上下拼接，`axis=1` 左右拼接
- 输入不合法时抛 `std::logic_error`（维度不匹配、求奇异矩阵的逆、下标越界……）
- 空矩阵要单独考虑（例如空矩阵的行列式约定为 1）