#pragma once 

#include <vector>
#include <cstddef>

namespace rm {
    struct Vector {
        double x;
        double y;
    };

    double length(const Vector& v);
    double distance(const Vector& a, const Vector& b);
    double dot(const Vector& a, const Vector& b);
    Vector scale(const Vector& a, double k);
    Vector normalize(const Vector& v);
    double angleBetween(const Vector& a, const Vector& b);
    
    using Matrix = std::vector<std::vector<double>>;

    // создание
    Matrix zeros(size_t n, size_t m);
    Matrix ones(size_t n, size_t m);
    Matrix random(size_t n, size_t m, double min, double max);

    // вывод
    void show(const Matrix& matrix);

    // арифметика
    Matrix multiply(const Matrix& matrix, double c);
    Matrix multiply(const Matrix& m1, const Matrix& m2);
    Matrix sum(const Matrix& matrix, double c);
    Matrix sum(const Matrix& m1, const Matrix& m2);

    // линейная алгебра
    Matrix transpose(const Matrix& matrix);
    Matrix minor(const Matrix& matrix, size_t n, size_t m);
    double determinant(const Matrix& matrix);
    Matrix inverse(const Matrix& matrix);

    // прочее
    Matrix concatenate(const Matrix& m1, const Matrix& m2, int axis = 0);

    // элементарные преобразования строк (ERO)
    Matrix ero_swap(const Matrix& matrix, size_t r1, size_t r2);
    Matrix ero_multiply(const Matrix& matrix, size_t r, double c);
    Matrix ero_sum(const Matrix& matrix, size_t r1, double c, size_t r2);

    Matrix upper_triangular(const Matrix& matrix);



}