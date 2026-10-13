#include "matrix.hpp"
#include <cmath>
#include <random>
#include <iomanip>
#include <iostream>
#include <stdexcept>
namespace rm {
Matrix zeros(size_t n, size_t m) {
    return Matrix(n, std::vector<double>(m, 0.0));
}
Matrix ones(size_t n, size_t m) {
    return Matrix(n, std::vector<double>(m, 1.0));
}
Matrix random(size_t n, size_t m, double min, double max) {
    if (min > max) throw std::logic_error("min > max");
    Matrix res(n, std::vector<double>(m));
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(min, max);
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < m; ++j)
            res[i][j] = dist(gen);
    return res;
}
void show(const Matrix& matrix) {
    for (const auto& row : matrix) {
        for (double x : row)
            std::cout << std::fixed << std::setprecision(3)
                      << std::setw(8) << x << " ";
        std::cout << "\n";
    }
}
Matrix multiply(const Matrix& matrix, double c) {
    Matrix res = matrix;
    for (auto& row : res)
        for (double& x : row) x *= c;
    return res;
}
Matrix multiply(const Matrix& m1, const Matrix& m2) {
    if (m1.empty() || m2.empty()) return Matrix{};
    size_t r1 = m1.size(), c1 = m1[0].size();
    size_t r2 = m2.size(), c2 = m2[0].size();
    if (c1 != r2) throw std::logic_error("matrix dimension mismatch");
    Matrix res(r1, std::vector<double>(c2, 0.0));
    for (size_t i = 0; i < r1; ++i)
        for (size_t k = 0; k < c1; ++k)
            for (size_t j = 0; j < c2; ++j)
                res[i][j] += m1[i][k] * m2[k][j];
    return res;
}
Matrix sum(const Matrix& matrix, double c) {
    Matrix res = matrix;
    for (auto& row : res)
        for (double& x : row) x += c;
    return res;
}
Matrix sum(const Matrix& m1, const Matrix& m2) {
    if (m1.size() != m2.size()) throw std::logic_error("row mismatch");
    if (!m1.empty() && m1[0].size() != m2[0].size())
        throw std::logic_error("col mismatch");

    Matrix res = m1;
    for (size_t i = 0; i < m1.size(); ++i)
        for (size_t j = 0; j < m1[i].size(); ++j)
            res[i][j] += m2[i][j];
    return res;
}
Matrix transpose(const Matrix& matrix) {
    if (matrix.empty()) return Matrix{};
    size_t r = matrix.size(), c = matrix[0].size();
    Matrix res(c, std::vector<double>(r, 0.0));
    for (size_t i = 0; i < r; ++i)
        for (size_t j = 0; j < c; ++j)
            res[j][i] = matrix[i][j];
    return res;
}
Matrix minor(const Matrix& matrix, size_t n, size_t m) {
    Matrix res;
    for (size_t i = 0; i < matrix.size(); ++i) {
        if (i == n) continue;
        std::vector<double> row;
        for (size_t j = 0; j < matrix[i].size(); ++j) {
            if (j == m) continue;
            row.push_back(matrix[i][j]);
        }
        res.push_back(row);
    }
    return res;
}
double determinant(const Matrix& matrix) {
    if (matrix.empty()) return 1.0;
    size_t n = matrix.size();
    if (n == 1) return matrix[0][0];
    double det = 0.0;
    for (size_t j = 0; j < n; ++j) {
        double sign = (j % 2 == 0) ? 1.0 : -1.0;
        det += sign * matrix[0][j] * determinant(minor(matrix, 0, j));
    }
    return det;
}
Matrix inverse(const Matrix& matrix) {
    double det = determinant(matrix);
    if (std::abs(det) < 1e-12)
        throw std::logic_error("singular matrix, cannot invert");
    size_t n = matrix.size();
    Matrix adj(n, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            double sign = ((i + j) % 2 == 0) ? 1.0 : -1.0;
            adj[j][i] = sign * determinant(minor(matrix, i, j));
        }
    }
    return multiply(adj, 1.0 / det);
}
Matrix concatenate(const Matrix& m1, const Matrix& m2, int axis) {
    if (axis == 0) {
        if (!m1.empty() && !m2.empty() && m1[0].size() != m2[0].size())
            throw std::logic_error("column mismatch for axis=0");
        Matrix res = m1;
        res.insert(res.end(), m2.begin(), m2.end());
        return res;
    }
    else if (axis == 1) {
        if (m1.size() != m2.size())
            throw std::logic_error("row mismatch for axis=1");
        Matrix res = m1;
        for (size_t i = 0; i < res.size(); ++i) {
            res[i].insert(res[i].end(), m2[i].begin(), m2[i].end());
        }
        return res;
    }
    else {
        throw std::logic_error("axis must be 0 or 1");
    }
}
Matrix ero_swap(const Matrix& matrix, size_t r1, size_t r2) {
    if (r1 >= matrix.size() || r2 >= matrix.size())
        throw std::logic_error("row index out of range");
    Matrix res = matrix;
    std::swap(res[r1], res[r2]);
    return res;
}

Matrix ero_multiply(const Matrix& matrix, size_t r, double c) {
    if (r >= matrix.size()) throw std::logic_error("row index out of range");
    Matrix res = matrix;
    for (double& x : res[r]) x *= c;
    return res;
}
Matrix ero_sum(const Matrix& matrix, size_t r1, double c, size_t r2) {
    if (r1 >= matrix.size() || r2 >= matrix.size())
        throw std::logic_error("row index out of range");
    Matrix res = matrix;
    for (size_t j = 0; j < res[r1].size(); ++j)
        res[r1][j] += c * res[r2][j];
    return res;
}
Matrix upper_triangular(const Matrix& matrix) {
    if (matrix.empty()) return Matrix{};
    size_t n = matrix.size(), m = matrix[0].size();
    Matrix res = matrix;
    for (size_t col = 0; col < std::min(n, m); ++col) {
        size_t pivot = col;
        while (pivot < n && std::abs(res[pivot][col]) < 1e-12) ++pivot;
        if (pivot == n) continue;
        if (pivot != col)
            res = ero_swap(res, col, pivot);
        for (size_t r = col + 1; r < n; ++r) {
            double factor = -res[r][col] / res[col][col];
            res = ero_sum(res, r, factor, col);
        }
    }
    return res;
}
}