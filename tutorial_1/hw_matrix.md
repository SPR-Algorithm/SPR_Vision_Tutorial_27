# 4. 加分项: Matrix Operations (optional)

To be implemented in addition to passing all 6 vector interfaces. Does not affect whether Stage 1 (第一阶段 dì-yī jiēduàn) passes — it only adds points to the overall grade (总评 zǒngpíng).

**Type**: Reuse the `rm::Matrix` defined in Section 1 (第1节 dì-yī jié).

```cpp
using Matrix = std::vector<std::vector<double>>;
```

## Interfaces (接口 jiēkǒu)

| Interface | Signature |
|---|---|
| zeros | `Matrix zeros(size_t n, size_t m)` |
| ones | `Matrix ones(size_t n, size_t m)` |
| random | `Matrix random(size_t n, size_t m, double min, double max)` |
| show | `void show(const Matrix& matrix)` |
| multiply (scalar multiplication / 数乘 shùchéng) | `Matrix multiply(const Matrix& matrix, double c)` |
| multiply (matrix multiplication / 矩阵乘 jǔzhèn chéng) | `Matrix multiply(const Matrix& m1, const Matrix& m2)` |
| sum (add constant / 加常数 jiā chángshù) | `Matrix sum(const Matrix& matrix, double c)` |
| sum (matrix addition / 矩阵加 jǔzhèn jiā) | `Matrix sum(const Matrix& m1, const Matrix& m2)` |
| transpose (转置 zhuǎnzhì) | `Matrix transpose(const Matrix& matrix)` |
| minor (余子式 yúzǐshì) | `Matrix minor(const Matrix& matrix, size_t n, size_t m)` |
| determinant (行列式 hánglièshì) | `double determinant(const Matrix& matrix)` |
| inverse (逆矩阵 nì jǔzhèn) | `Matrix inverse(const Matrix& matrix)` |
| concatenate (拼接 pīnjiē) | `Matrix concatenate(const Matrix& m1, const Matrix& m2, int axis = 0)` |
| ero_swap (行交换 / row swap, xíng jiāohuàn) | `Matrix ero_swap(const Matrix& matrix, size_t r1, size_t r2)` |
| ero_multiply (行数乘 / row scalar mult, xíng shùchéng) | `Matrix ero_multiply(const Matrix& matrix, size_t r, double c)` |
| ero_sum (行相加 / row addition, xíng xiāngjiā) | `Matrix ero_sum(const Matrix& matrix, size_t r1, double c, size_t r2)` |
| upper_triangular (上三角矩阵 shàng sānjiǎo jǔzhèn) | `Matrix upper_triangular(const Matrix& matrix)` |

## Constraints (约束 yuēshù)

- Determinant (行列式) must use cofactor/minor expansion (余子式展开 yúzǐshì zhǎnkāi).
- Random numbers (随机数 suíjīshù) must use `<random>`.
- `show` must use `<iomanip>`; each element fixed to 3 decimal places (固定3位小数 gùdìng sān wèi xiǎoshù), aligned output (对齐输出 duìqí shūchū).
- `concatenate`'s `axis=0` stacks vertically/top-bottom (上下拼接 shàngxià pīnjiē), `axis=1` joins side by side/left-right (左右拼接 zuǒyòu pīnjiē).
- On invalid input, throw `std::logic_error` (dimension mismatch/维度不匹配 wéidù bù pǐpèi, inverse of a singular matrix/求奇异矩阵的逆 qiú qíyì jǔzhèn de nì, out-of-bounds index/下标越界 xiàbiāo yuèjiè, etc.).
- Empty matrices (空矩阵 kōng jǔzhèn) need special handling — e.g., the determinant of an empty matrix is conventionally (约定 yuēdìng) defined as 1.

# 5. Reference File Structure (参考文件结构 cānkǎo wénjiàn jiégòu)

Recommended layout: declarations (声明 shēngmíng) go in `include/`, implementation (实现 shíxiàn) goes in `src/`, self-tests (自测 zìcè) go in `test/`. File names aren't mandatory, but two things are required: header/implementation separation (头文件与实现分离 tóuwénjiàn yǔ shíxiàn fēnlí), and `include/` being directly `#include`-able.

```
homework_1/
├── CMakeLists.txt                # add_library + target_include_directories
├── include/                       (#pragma once)
│   ├── vector.hpp                
│   └── matrix.hpp                
├── src/                          
│   ├── vector.cpp
│   └── matrix.cpp                # bonus item (加分项)
└── test/                         # self-test cases (optional, content not checked / 自测用例，选做，不检查内容)
    ├── test_vector.cpp
    └── test_matrix.cpp           # bonus item
```

## Correspondence (对应关系 duìyìng guānxì)

| Directory / File | Purpose (作用 zuòyòng) | Location in Assignment |
|---|---|---|
| `include/*.hpp` | Interface declarations, boundary conventions (接口声明、边界约定 jiēkǒu shēngmíng, biānjiè yuēdìng) | §1 Interface format |
| `src/*.cpp` | Concrete implementation (具体实现 jùtǐ shíxiàn) | §1 / §4 Constraints |
| `CMakeLists.txt` | Build the library and expose `include/` (造库并暴露 zào kù bìng bàolù) | §2.3 CMake |
| `test/` | Write your own test cases to verify (自己写用例验证 zìjǐ xiě yònglì yànzhèng) (optional) | §3 Acceptance checklist (验收清单 yànshōu qīngdān) |

## Key points (要点 yàodiǎn)

- `include/` must be exposed via `target_include_directories(<lib> PUBLIC include)`, so that users only need to write `#include "vector.hpp"`.
- `matrix.hpp` needs `rm::Matrix` / `rm::Vector`, so the bonus library must link against it: `target_link_libraries(rm_matrix PUBLIC rm_vector)`.
- Only keep source code and `CMakeLists.txt` in the root directory (根目录 gēn mùlù). `build/`, `*.o` / `*.a` should all be blocked by `.gitignore` — do not commit (提交 tíjiāo) them.