// Tests were made by AI
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "qvec.hpp"

#define CHECK(expr)                                                        \
    do {                                                                   \
        if (!(expr)) {                                                     \
            std::cerr << "FAIL: " #expr " (line " << __LINE__ << ")\n";    \
            return 1;                                                      \
        }                                                                  \
    } while (0)

#define CHECK_THROWS(stmt)                                                 \
    do {                                                                   \
        bool threw_ = false;                                               \
        try {                                                              \
            stmt;                                                          \
        } catch (const std::logic_error&) {                                \
            threw_ = true;                                                 \
        }                                                                  \
        if (!threw_) {                                                     \
            std::cerr << "FAIL: expected throw from " #stmt                \
                       << " (line " << __LINE__ << ")\n";                  \
            return 1;                                                      \
        }                                                                  \
    } while (0)

static bool nearlyEqual(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) < eps;
}

int main() {
    using rm::Matrix;

    // zeros / ones
    {
        Matrix z = rm::zeros(2, 3);
        CHECK(z.size() == 2);
        CHECK(z[0].size() == 3);
        for (const auto& row : z)
            for (double v : row)
                CHECK(nearlyEqual(v, 0.0));

        Matrix o = rm::ones(3, 2);
        CHECK(o.size() == 3);
        CHECK(o[0].size() == 2);
        for (const auto& row : o)
            for (double v : row)
                CHECK(nearlyEqual(v, 1.0));

        Matrix e = rm::zeros(0, 0);
        CHECK(e.empty());
    }

    // random
    {
        Matrix r = rm::random(4, 4, -5.0, 5.0);
        for (const auto& row : r)
            for (double v : row)
                CHECK(v >= -5.0 && v <= 5.0);
    }

    // multiply (scalar)
    {
        const Matrix a{{1, 2}, {3, 4}};
        const Matrix expected{{2, 4}, {6, 8}};
        CHECK(rm::multiply(a, 2.0) == expected);
    }

    // multiply (matrix)
    {
        const Matrix a{{1, 2, 3}, {4, 5, 6}};       // 2x3
        const Matrix b{{7, 8}, {9, 10}, {11, 12}};  // 3x2
        const Matrix expected{{58, 64}, {139, 154}};
        CHECK(rm::multiply(a, b) == expected);

        const Matrix c{{1, 2}, {3, 4}};  // 2x2, несовместимо с a (2x3)
        CHECK_THROWS(rm::multiply(a, c));
    }

    // sum (scalar)
    {
        const Matrix a{{1, 2}, {3, 4}};
        const Matrix expected{{11, 12}, {13, 14}};
        CHECK(rm::sum(a, 10.0) == expected);
    }

    // sum (matrix)
    {
        const Matrix a{{1, 2}, {3, 4}};
        const Matrix b{{5, 6}, {7, 8}};
        const Matrix expected{{6, 8}, {10, 12}};
        CHECK(rm::sum(a, b) == expected);

        const Matrix c{{1, 2, 3}};
        CHECK_THROWS(rm::sum(a, c));
    }

    // transpose
    {
        const Matrix a{{1, 2, 3}, {4, 5, 6}};  // 2x3
        const Matrix expected{{1, 4}, {2, 5}, {3, 6}};
        CHECK(rm::transpose(a) == expected);
    }

    // minor
    {
        const Matrix a{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
        const Matrix expected{{1, 3}, {7, 9}};
        CHECK(rm::minor(a, 1, 1) == expected);

        CHECK_THROWS(rm::minor(a, 10, 0));
    }

    // determinant
    {
        CHECK(nearlyEqual(rm::determinant(Matrix{}), 1.0));       // пустая матрица
        CHECK(nearlyEqual(rm::determinant(Matrix{{5}}), 5.0));    // 1x1

        const Matrix a2{{1, 2}, {3, 4}};
        CHECK(nearlyEqual(rm::determinant(a2), -2.0));

        const Matrix a3{{6, 1, 1}, {4, -2, 5}, {2, 8, 7}};
        CHECK(nearlyEqual(rm::determinant(a3), -306.0));

        CHECK_THROWS(rm::determinant(Matrix{{1, 2, 3}, {4, 5, 6}}));  // не квадратная
    }

    // inverse
    {
        const Matrix a{{4, 7}, {2, 6}};
        const Matrix inv = rm::inverse(a);
        const Matrix identity = rm::multiply(a, inv);
        CHECK(nearlyEqual(identity[0][0], 1.0));
        CHECK(nearlyEqual(identity[0][1], 0.0));
        CHECK(nearlyEqual(identity[1][0], 0.0));
        CHECK(nearlyEqual(identity[1][1], 1.0));

        const Matrix singular{{1, 2}, {2, 4}};
        CHECK_THROWS(rm::inverse(singular));
    }

    // concatenate
    {
        const Matrix a{{1, 2}, {3, 4}};
        const Matrix b{{5, 6}, {7, 8}};

        const Matrix expectedVert{{1, 2}, {3, 4}, {5, 6}, {7, 8}};
        CHECK(rm::concatenate(a, b, 0) == expectedVert);

        const Matrix expectedHoriz{{1, 2, 5, 6}, {3, 4, 7, 8}};
        CHECK(rm::concatenate(a, b, 1) == expectedHoriz);

        CHECK_THROWS(rm::concatenate(a, Matrix{{1, 2, 3}}, 0));
    }

    // ero_swap
    {
        const Matrix a{{1, 2}, {3, 4}, {5, 6}};
        const Matrix expected{{5, 6}, {3, 4}, {1, 2}};
        CHECK(rm::ero_swap(a, 0, 2) == expected);
    }

    // ero_multiply
    {
        const Matrix a{{1, 2}, {3, 4}};
        const Matrix expected{{1, 2}, {6, 8}};
        CHECK(rm::ero_multiply(a, 1, 2.0) == expected);
    }

    // ero_sum
    {
        const Matrix a{{1, 2}, {3, 4}};
        const Matrix expected{{7, 10}, {3, 4}};  // row0 += 2*row1
        CHECK(rm::ero_sum(a, 0, 2.0, 1) == expected);
    }

    // upper_triangular
    {
        const Matrix a{{2, 4, -2}, {4, 9, -3}, {-2, -3, 7}};
        const Matrix res = rm::upper_triangular(a);
        for (size_t i = 0; i < res.size(); ++i)
            for (size_t j = 0; j < i; ++j)
                CHECK(nearlyEqual(res[i][j], 0.0, 1e-6));
    }

    std::cout << "all checks passed\n";
    return 0;
}