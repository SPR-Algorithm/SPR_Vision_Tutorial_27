#include "qvec.hpp"
#include <cmath>
#include <vector>
#include <cstddef>
#include <random>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <algorithm>

double getRandomDouble(double min, double max);   // forward declare

namespace rm {    

    double length(const Vector& v) {
        return std::sqrt((v.x*v.x) + (v.y*v.y));
    }

    double distance(const Vector& a, const Vector& b) {
        return length(Vector{a.x - b.x, a.y-b.y});
    }

    double dot(const Vector& a, const Vector& b) {
        return ((a.x * b.x) + (a.y * b.y));        
    }

    Vector scale(const Vector& v, double k) {
        return Vector{v.x*k, v.y*k};
    }

    Vector normalize(const Vector& v) {
        double l = length(v);
        return Vector{v.x/l, v.y/l};
    }

    double angleBetween(const Vector& a, const Vector& b) {
        return std::acos(dot(a, b)/(length(a)*length(b)))  * 180.0 / M_PI;
    }

    Matrix zeros(size_t n, size_t m) {
    return Matrix(n, std::vector<double>(m, 0.0));
}
Matrix ones(size_t n, size_t m) {
    return Matrix(n, std::vector<double>(m, 1.0));
}
Matrix random(size_t n, size_t m, double min, double max) {
    Matrix result(n, std::vector<double>(m, 0.0));
    for (auto& row : result) {
        for (auto& val : row) {
            val = getRandomDouble(min, max);
        }
    }
    return result;
}

void show(const Matrix& matrix) {
    std::cout << std::fixed << std::setprecision(3);
    for (const auto& row : matrix) {
        std::cout << "[ ";
        for (const auto& val : row) {
            std::cout << std::setw(8) << val << " ";
        }
        std::cout << "]\n";
    }
}

Matrix multiply(const Matrix& matrix, double c) {
    Matrix result = matrix;
    for (auto& row : result) {
        for (auto& val : row) {
            val *= c;
        }
    }
    return result;
}

Matrix multiply(const Matrix& m1, const Matrix& m2) {
    size_t n = m1.size();
    size_t k = m1.empty() ? 0 : m1[0].size();
    size_t m = m2.empty() ? 0 : m2[0].size();

    if (m2.size() != k) {
        throw std::logic_error("multiply: dimension mismatch");
    }

    Matrix result(n, std::vector<double>(m, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < m; ++j) {
            double sum = 0.0;
            for (size_t p = 0; p < k; ++p) {
                sum += m1[i][p] * m2[p][j];
            }
            result[i][j] = sum;
        }
    }
    return result;
}

Matrix sum(const Matrix& matrix, double c) {
    Matrix result = matrix;
    for (auto& row : result) {
        for (auto& val : row) {
            val += c;
        }
    }
    return result;
}

Matrix sum(const Matrix& m1, const Matrix& m2) {
    size_t n1 = m1.size();
    size_t c1 = m1.empty() ? 0 : m1[0].size();
    size_t n2 = m2.size();
    size_t c2 = m2.empty() ? 0 : m2[0].size();

    if (n1 != n2 || c1 != c2) {
        throw std::logic_error("sum: dimension mismatch");
    }

    Matrix result = zeros(n1, c1);
    for (size_t i = 0; i < n1; ++i) {
        for (size_t j = 0; j < c1; ++j) {
            result[i][j] = m1[i][j] + m2[i][j];
        }
    }
    return result;
}

Matrix transpose(const Matrix& matrix) {
    size_t x = matrix.size();
    size_t y = matrix.empty() ? 0 : matrix[0].size();
    Matrix result = zeros(y, x);
    for (size_t i = 0; i < x; ++i) {
        for (size_t j = 0; j < y; ++j) {
            result[j][i] = matrix[i][j];
        }
    }
    return result;
}

// n, m — индекс строки и столбца, которые нужно ВЫЧЕРКНУТЬ
Matrix minor(const Matrix& matrix, size_t n, size_t m) {
    size_t x = matrix.size();
    size_t y = matrix.empty() ? 0 : matrix[0].size();
    if (n >= x || m >= y) {
        throw std::logic_error("minor: index out of bounds");
    }

    Matrix result(x - 1, std::vector<double>(y - 1, 0.0));
    size_t ri = 0;
    for (size_t i = 0; i < x; ++i) {
        if (i == n) continue;
        size_t rj = 0;
        for (size_t j = 0; j < y; ++j) {
            if (j == m) continue;
            result[ri][rj] = matrix[i][j];
            ++rj;
        }
        ++ri;
    }
    return result;
}

double determinant(const Matrix& matrix) {
    size_t x = matrix.size();
    size_t y = matrix.empty() ? 0 : matrix[0].size();
    if (x != y) {
        throw std::logic_error("determinant: matrix must be square");
    }
    if (x == 0) {
        return 1.0;  // пустая матрица — по условию
    }
    if (x == 1) {
        return matrix[0][0];
    }

    double det = 0.0;
    double sign = 1.0;
    for (size_t j = 0; j < x; ++j) {
        det += sign * matrix[0][j] * determinant(minor(matrix, 0, j));
        sign = -sign;
    }
    return det;
}

Matrix inverse(const Matrix& matrix) {
    double det = determinant(matrix);
    if (det == 0.0) {
        throw std::logic_error("inverse: matrix is singular");
    }
    size_t n = matrix.size();
    Matrix cofactors(n, std::vector<double>(n, 0.0));
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            double sign = ((i + j) % 2 == 0) ? 1.0 : -1.0;
            cofactors[i][j] = sign * determinant(minor(matrix, i, j));
        }
    }
    return multiply(transpose(cofactors), 1.0 / det);
}

Matrix concatenate(const Matrix& m1, const Matrix& m2, int axis) {
    size_t n1 = m1.size(), c1 = m1.empty() ? 0 : m1[0].size();
    size_t n2 = m2.size(), c2 = m2.empty() ? 0 : m2[0].size();

    if (axis == 0) {
        if (c1 != c2) throw std::logic_error("concatenate: column mismatch");
        Matrix result = m1;
        result.insert(result.end(), m2.begin(), m2.end());
        return result;
    } else if (axis == 1) {
        if (n1 != n2) throw std::logic_error("concatenate: row mismatch");
        Matrix result(n1);
        for (size_t i = 0; i < n1; ++i) {
            result[i] = m1[i];
            result[i].insert(result[i].end(), m2[i].begin(), m2[i].end());
        }
        return result;
    }
    throw std::logic_error("concatenate: invalid axis");
}

Matrix ero_swap(const Matrix& matrix, size_t r1, size_t r2) {
    if (r1 >= matrix.size() || r2 >= matrix.size()) {
        throw std::logic_error("ero_swap: index out of bounds");
    }
    Matrix result = matrix;
    std::swap(result[r1], result[r2]);
    return result;
}

Matrix ero_multiply(const Matrix& matrix, size_t r, double c) {
    if (r >= matrix.size()) {
        throw std::logic_error("ero_multiply: index out of bounds");
    }
    Matrix result = matrix;
    for (auto& val : result[r]) {
        val *= c;
    }
    return result;
}

Matrix ero_sum(const Matrix& matrix, size_t r1, double c, size_t r2) {
    if (r1 >= matrix.size() || r2 >= matrix.size()) {
        throw std::logic_error("ero_sum: index out of bounds");
    }
    Matrix result = matrix;
    for (size_t j = 0; j < result[r1].size(); ++j) {
        result[r1][j] += c * result[r2][j];
    }
    return result;
}

Matrix upper_triangular(const Matrix& matrix) {
    Matrix result = matrix;
    size_t n = result.size();
    size_t m = n == 0 ? 0 : result[0].size();

    size_t pivotRow = 0;
    for (size_t col = 0; col < m && pivotRow < n; ++col) {
        size_t sel = pivotRow;
        while (sel < n && result[sel][col] == 0.0) ++sel;
        if (sel == n) continue;
        if (sel != pivotRow) result = ero_swap(result, pivotRow, sel);
        for (size_t i = pivotRow + 1; i < n; ++i) {
            if (result[i][col] != 0.0) {
                double factor = -result[i][col] / result[pivotRow][col];
                result = ero_sum(result, i, factor, pivotRow);
            }
        }
        ++pivotRow;
    }
    return result;
}

}  // namespace rm

double getRandomDouble(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(min, max);
    return dis(gen);
}