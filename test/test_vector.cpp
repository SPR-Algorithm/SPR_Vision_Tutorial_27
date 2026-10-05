#include <gtest/gtest.h>
#include "vector2d.hpp"

TEST(VectorTest, Length) {
    rm::Vector v1 = {3.0, 4.0};
    EXPECT_DOUBLE_EQ(rm::length(v1), 5.0);
    rm::Vector v2 = {0.0, 0.0};
    EXPECT_DOUBLE_EQ(rm::length(v2), 0.0);
}

TEST(VectorTest, Distance) {
    rm::Vector a = {1.0, 2.0};
    rm::Vector b = {4.0, 6.0};
    EXPECT_DOUBLE_EQ(rm::distance(a, b), 5.0);
}

TEST(VectorTest, DotProduct) {
    rm::Vector a = {1.0, 2.0};
    rm::Vector b = {3.0, 4.0};
    EXPECT_DOUBLE_EQ(rm::dot(a, b), 11.0);
}

TEST(VectorTest, Scale) {
    rm::Vector v = {1.0, 2.0};
    rm::Vector res = rm::scale(v, 2.0);
    EXPECT_DOUBLE_EQ(res[0], 2.0);
    EXPECT_DOUBLE_EQ(res[1], 4.0);
}

TEST(VectorTest, Normalize) {
    rm::Vector v = {3.0, 4.0};
    rm::Vector res = rm::normalize(v);
    EXPECT_DOUBLE_EQ(rm::length(res), 1.0);
}

TEST(VectorTest, AngleBetween) {
    rm::Vector a = {1.0, 0.0};
    rm::Vector b = {0.0, 1.0};
    EXPECT_NEAR(rm::angleBetween(a, b), 3.1415926 / 2, 1e-6);
}