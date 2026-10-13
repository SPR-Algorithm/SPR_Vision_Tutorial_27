#include "matrix.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
using namespace rm;
void print(const std::string& name, const Matrix& m) {
    std::cout << "=== " << name << " ===\n";
    show(m);
    std::cout << "\n";
}
int main() {
    Matrix z = zeros(2, 3);
    print("zeros(2,3)", z);

    Matrix o = ones(2, 3);
    print("ones(2,3)", o);

    Matrix r = random(2, 3, 0.0, 1.0);
    print("random(2,3,0,1)", r);

    Matrix a = {{1, 2}, {3, 4}};
    Matrix b = {{5, 6}, {7, 8}};

    print("a", a);
    print("b", b);

    print("a + b", sum(a, b));
    print("a + 10", sum(a, 10.0));
    print("a * 2", multiply(a, 2.0));

    Matrix c = {{1, 2}, {3, 4}};
    Matrix d = {{1}, {2}};
    print("c * d", multiply(c, d));

    print("transpose(a)", transpose(a));

    Matrix m3 = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    print("m3", m3);
    print("minor(m3, 0, 0)", minor(m3, 0, 0));

    Matrix det_test = {{1, 2}, {3, 4}};
    std::cout << "det([[1,2],[3,4]]) = " << determinant(det_test) << "\n\n";

    Matrix empty;
    std::cout << "det(empty) = " << determinant(empty) << "\n\n";

    Matrix inv_test = {{4, 7}, {2, 6}};
    print("inv([[4,7],[2,6]])", inverse(inv_test));

    Matrix e = {{1, 2}, {3, 4}};
    Matrix f = {{5, 6}, {7, 8}};

    print("concatenate(e, f, 0) 上下拼", concatenate(e, f, 0));
    print("concatenate(e, f, 1) 左右拼", concatenate(e, f, 1));

    print("ero_swap(e, 0, 1)", ero_swap(e, 0, 1));
    print("ero_multiply(e, 0, 2)", ero_multiply(e, 0, 2));
    print("ero_sum(e, 1, 2, 0)", ero_sum(e, 1, 2.0, 0));

    Matrix ut = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    print("upper_triangular(m3)", upper_triangular(ut));

    try {
        Matrix bad1 = {{1, 2}, {3, 4}};
        Matrix bad2 = {{1, 2, 3}};
        sum(bad1, bad2);
        std::cout << "ERROR: 应该抛异常却没有抛\n";
    } catch (const std::logic_error& e) {
        std::cout << "[OK] 捕获到异常: " << e.what() << "\n";
    }

    try {
        Matrix singular = {{1, 2}, {2, 4}};
        inverse(singular);
        std::cout << "ERROR: 应该抛异常却没有抛\n";
    } catch (const std::logic_error& e) {
        std::cout << "[OK] 捕获到异常: " << e.what() << "\n";
    }

    try {
        Matrix m = {{1, 2}, {3, 4}};
        ero_swap(m, 0, 5);
        std::cout << "ERROR: 应抛异常却没有抛\n";
    } catch (const std::logic_error& e) {
        std::cout << "捕获到异常: " << e.what() << "\n";
    }
    std::cout << "\n测试完成。\n";
    return 0;
}