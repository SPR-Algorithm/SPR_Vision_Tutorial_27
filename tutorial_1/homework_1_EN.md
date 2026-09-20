# Stage 1 Assignment: 2D Vector Math Library

Corresponding curriculum: [`tutorial_1.md`](tutorial_1.md) (git workflow is in curriculum §2.3)

---

## 1. Task

Write a simple **2D vector math library**: compute the distance, magnitude, dot product, and scaling of 2D vectors, and implement normalization and the angle between vectors.

**Type definitions**

```cpp
#include <vector>

namespace rm {

using Vector = std::vector<double>;                // n-dimensional vector; for 2D, size() == 2
using Matrix = std::vector<std::vector<double>>;   // matrix

}  // namespace rm
```

**Interface format** (follow this for parameters and return values)

| Interface      | Signature                                                |
| -------------- | ---------------------------------------------------------- |
| `length`       | `double length(const Vector& v)`                          |
| `distance`     | `double distance(const Vector& a, const Vector& b)`       |
| `dot`          | `double dot(const Vector& a, const Vector& b)`             |
| `scale`        | `Vector scale(const Vector& v, double k)`                 |
| `normalize`    | `Vector normalize(const Vector& v)`                        |
| `angleBetween` | `double angleBetween(const Vector& a, const Vector& b)`   |

**Boundary checks**: each interface must validate its input before computing; a zero vector must never cause a division by zero; vectors involved in an operation must have matching dimensions; `NaN` / `inf` must never occur anywhere.

## 2. Requirements

1. **Git**: complete `clone`, `commit`, and `push`; commit messages must follow Conventional Commits.
2. **Deliverable**: submit only the library itself (header + implementation + `CMakeLists.txt`) — no `main()` needed.
3. **CMake**: write your own `CMakeLists.txt`; use `add_library` to build the math code as a static library, and expose `include/` to consumers via `target_include_directories`.
4. **Conventions**: use `#pragma once` in headers; keep declarations and implementations separate.

## 3. Acceptance Checklist

**Deliverables**

- [ ] The assignment is submitted on **your own branch**, `feat/<your-name>`, **not** committed directly to `main`
- [ ] Before pushing, you've synced the latest content from `main` (`git rebase main`)
- [ ] Commit messages follow Conventional Commits (`feat` / `fix` / `build` / `docs` …)
- [ ] The repository does **not** contain committed `build/` directories or build artifacts (`.o` / `.a` / `.so`); there is a `.gitignore`

## 4. Bonus: Matrix Operations (Optional)

> To be implemented in addition to, once all 6 vector interfaces pass. **Does not affect whether Stage 1 passes** — it only adds to the overall evaluation.

**Type**: reuses `rm::Matrix` as defined in Section 1

```cpp
using Matrix = std::vector<std::vector<double>>;
```

**Interfaces**

| Interface                | Signature                                                               |
| -------------------------- | -------------------------------------------------------------------------- |
| `zeros`                   | `Matrix zeros(size_t n, size_t m)`                                        |
| `ones`                    | `Matrix ones(size_t n, size_t m)`                                         |
| `random`                  | `Matrix random(size_t n, size_t m, double min, double max)`               |
| `show`                    | `void show(const Matrix& matrix)`                                         |
| `multiply` (scalar)       | `Matrix multiply(const Matrix& matrix, double c)`                        |
| `multiply` (matrix)       | `Matrix multiply(const Matrix& m1, const Matrix& m2)`                    |
| `sum` (add constant)      | `Matrix sum(const Matrix& matrix, double c)`                              |
| `sum` (matrix addition)   | `Matrix sum(const Matrix& m1, const Matrix& m2)`                          |
| `transpose`               | `Matrix transpose(const Matrix& matrix)`                                  |
| `minor`                   | `Matrix minor(const Matrix& matrix, size_t n, size_t m)`                  |
| `determinant`             | `double determinant(const Matrix& matrix)`                                |
| `inverse`                 | `Matrix inverse(const Matrix& matrix)`                                    |
| `concatenate`             | `Matrix concatenate(const Matrix& m1, const Matrix& m2, int axis = 0)`   |
| `ero_swap`                | `Matrix ero_swap(const Matrix& matrix, size_t r1, size_t r2)`             |
| `ero_multiply`            | `Matrix ero_multiply(const Matrix& matrix, size_t r, double c)`           |
| `ero_sum`                 | `Matrix ero_sum(const Matrix& matrix, size_t r1, double c, size_t r2)`   |
| `upper_triangular`        | `Matrix upper_triangular(const Matrix& matrix)`                           |

**Constraints**

- The determinant must be computed **via cofactor expansion**
- Random numbers must use `<random>`
- `show` must use `<iomanip>`, printing each element with a fixed **3 decimal places**, aligned
- For `concatenate`, `axis=0` concatenates top-to-bottom, `axis=1` concatenates left-to-right
- Invalid input must throw `std::logic_error` (dimension mismatch, inverting a singular matrix, out-of-bounds index, etc.)
- Empty matrices need special handling (e.g. the determinant of an empty matrix is defined as 1)
