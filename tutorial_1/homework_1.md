# 第一阶段作业：二维向量运算库

对应教案：[`tutorial_1.md`](tutorial_1.md)（git 工作流见教案 §2.3）

---

## 1. 题目

写一个简单的**n维向量运算库**：计算n维向量的距离、模长、点乘、缩放，实现单位化与夹角。

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

---

## 5. 参考文件结构

> 推荐布局：**声明放 `include/`、实现放 `src/`、自测放 `test/`**。文件名不强制，
> 但「头文件与实现分离、`include/` 能直接被 `#include` 到」这两点必须满足。

```text
homework_1/
├── CMakeLists.txt                # add_library + target_include_directories
├── include/                       |（#pragma once）
│   ├── vector.hpp                # 
│   └── matrix.hpp                # 
├── src/                          # 
│   ├── vector.cpp
│   └── matrix.cpp                # 加分项
└── test/                         # 自测用例（选做，不检查内容）
    ├── test_vector.cpp
    └── test_matrix.cpp           # 加分项
```

**对应关系**

| 目录 / 文件      | 作用                   | 作业中的位置 |
| ---------------- | ---------------------- | ------------ |
| `include/*.hpp`  | 接口声明、边界约定     | §1 接口格式  |
| `src/*.cpp`      | 具体实现               | §1 / §4 约束 |
| `CMakeLists.txt` | 造库并暴露 `include/`  | §2.3 CMake   |
| `test/`          | 自己写用例验证（选做） | §3 验收清单  |

**要点**

- `include/` 用 `target_include_directories(<lib> PUBLIC include)` 暴露出去，使用者只写 `#include "vector.hpp"`。
- `matrix.hpp` 里要用到 `rm::Matrix` / `rm::Vector`，所以加分项那个库要 `target_link_libraries(rm_matrix PUBLIC rm_vector)`。
- 根目录只留源码与 `CMakeLists.txt`：`build/`、`*.o` / `*.a` 一律交给 `.gitignore` 挡住，不要提交。