// matrix.cpp
#include "matrix.hpp"
#include <stdexcept>

Matrix::Matrix(int r, int c) : rows(r), cols(c), data(r * c, 0.0) {}

double Matrix::get(int r, int c) const {
    if (r < 0 || r >= rows || c <0 || c >= cols) {
        throw std::out_of_range("Index out of matrix range");
    }
    return data[r * cols + c];
}

void Matrix::set(int r, int c, double val) {
    if (r < 0 || r >= rows || c <0 || c >= cols) {
        throw std::out_of_range("Index out of matrix range");
    }
    data[r * cols + c] = val;
}

Matrix Matrix::add(const Matrix& other) const {
    if (rows != other.rows || cols != other.cols) {
        throw std::invalid_argument("Matrix dimension mismatch for add");
    }
    Matrix res(rows, cols);
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < cols; j++){
            res.set(i,j, get(i,j) + other.get(i,j));
        }
    }
    return res;
}

Matrix Matrix::multiply(const Matrix& other) const {
    if(cols != other.rows) {
        throw std::invalid_argument("Dimension mismatch for multiply");
    }
    Matrix res(rows, other.cols);
    for(int i = 0; i < rows; i++){
        for(int j = 0; j < other.cols; j++){
            double sum = 0.0;
            for(int k = 0; k < cols; k++){
                sum += get(i,k) * other.get(k,j);
            }
            res.set(i,j, sum);
        }
    }
    return res;
}

int Matrix::getRows() const { return rows; }
int Matrix::getCols() const { return cols; }