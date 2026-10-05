// test_matrix.cpp
#include <gtest/gtest.h>
#include "matrix.hpp"

TEST(MatrixTest, AddMatrix) {
    Matrix a(2,2);
    Matrix b(2,2);
    a.set(0,0,1); a.set(0,1,2);
    a.set(1,0,3); a.set(1,1,4);
    b.set(0,0,5); b.set(0,1,6);
    b.set(1,0,7); b.set(1,1,8);

    Matrix c = a.add(b);
    EXPECT_EQ(c.get(0,0), 6);
    EXPECT_EQ(c.get(0,1), 8);
    EXPECT_EQ(c.get(1,0), 10);
    EXPECT_EQ(c.get(1,1), 12);
}

TEST(MatrixTest, MultiplyMatrix) {
    Matrix a(2,3);
    Matrix b(3,2);
    a.set(0,0,1); a.set(0,1,2); a.set(0,2,3);
    a.set(1,0,4); a.set(1,1,5); a.set(1,2,6);
    b.set(0,0,7); b.set(0,1,8);
    b.set(1,0,9); b.set(1,1,10);
    b.set(2,0,11); b.set(2,1,12);

    Matrix res = a.multiply(b);
    EXPECT_EQ(res.get(0,0), 1*7 + 2*9 + 3*11);
    EXPECT_EQ(res.get(0,1), 1*8 + 2*10 + 3*12);
    EXPECT_EQ(res.get(1,0), 4*7 + 5*9 + 6*11);
    EXPECT_EQ(res.get(1,1), 4*8 + 5*10 + 6*12);
}

TEST(MatrixTest, DimensionCheck) {
    Matrix m(3,4);
    EXPECT_EQ(m.getRows(), 3);
    EXPECT_EQ(m.getCols(), 4);
}