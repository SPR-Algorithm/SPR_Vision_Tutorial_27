// Tests were made by AI
#include <cmath>
#include <iostream>
#include "qvec.hpp"

#define CHECK(expr)                                                        \
    do {                                                                   \
        if (!(expr)) {                                                     \
            std::cerr << "FAIL: " #expr " (line " << __LINE__ << ")\n";    \
            return 1;                                                      \
        }                                                                  \
    } while (0)

static bool nearlyEqual(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) < eps;
}

int main() {
    using rm::Vector;

    const Vector a{6.0, 8.0};
    const Vector b{3.0, 4.0};

    CHECK(nearlyEqual(rm::length(a), 10.0));
    CHECK(nearlyEqual(rm::length(b), 5.0));

    CHECK(nearlyEqual(rm::distance(a, b), 5.0));

    CHECK(nearlyEqual(rm::dot(a, b), 50.0));

    {
        const Vector c = rm::scale(a, 2.0);
        CHECK(nearlyEqual(c.x, 12.0));
        CHECK(nearlyEqual(c.y, 16.0));
    }

    {
        const Vector zero = rm::scale(a, 0.0);
        CHECK(nearlyEqual(zero.x, 0.0));
        CHECK(nearlyEqual(zero.y, 0.0));
    }

    {
        const Vector n = rm::normalize(a);
        CHECK(nearlyEqual(n.x, 0.6));
        CHECK(nearlyEqual(n.y, 0.8));
        CHECK(nearlyEqual(rm::length(n), 1.0));
    }

    CHECK(nearlyEqual(rm::angleBetween(a, b), 0.0));

    {
        const Vector px{1.0, 0.0};
        const Vector py{0.0, 1.0};
        CHECK(nearlyEqual(rm::angleBetween(px, py), 90.0));
    }

    {
        const Vector pos{1.0, 0.0};
        const Vector neg{-1.0, 0.0};
        CHECK(nearlyEqual(rm::angleBetween(pos, neg), 180.0));
    }

    std::cout << "all checks passed\n";
    return 0;
}