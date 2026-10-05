// matrix.hpp
#pragma once
#include <vector>

class Matrix {
private:
    int rows;
    int cols;
    std::vector<double> data;
public:
    Matrix(int r, int c);
    double get(int r, int c) const;
    void set(int r, int c, double val);
    Matrix add(const Matrix& other) const;
    Matrix multiply(const Matrix& other) const;
    int getRows() const;
    int getCols() const;
};