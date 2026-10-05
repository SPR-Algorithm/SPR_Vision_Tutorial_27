#include <iostream>
#include "vector2d.hpp"
#include "matrix.hpp"

int main() {
    // 示例：创建向量对象，调用类里面的方法（根据你 vector2d.hpp 的接口自行修改）
    Vector2d v1(3.0, 4.0);
    Vector2d v2(1.0, 2.0);

    std::cout << "v1 = (" << v1.x << ", " << v1.y << ")" << std::endl;
    std::cout << "v2 = (" << v2.x << ", " << v2.y << ")" << std::endl;

    Vector2d sum = v1 + v2;
    std::cout << "v1 + v2 = (" << sum.x << ", " << sum.y << ")" << std::endl;

    return 0;

    // 创建 2x2 矩阵 A
    Matrix A(2, 2);
    A.set(0, 0, 1.0);
    A.set(0, 1, 2.0);
    A.set(1, 0, 3.0);
    A.set(1, 1, 4.0);

    // 创建 2x2 矩阵 B
    Matrix B(2, 2);
    B.set(0, 0, 5.0);
    B.set(0, 1, 6.0);
    B.set(1, 0, 7.0);
    B.set(1, 1, 8.0);

    // 矩阵加法
    Matrix addResult = A.add(B);
    std::cout << "Matrix A + B result:\n";
    for (int i = 0; i < addResult.getRows(); ++i) {
        for (int j = 0; j < addResult.getCols(); ++j) {
            std::cout << addResult.get(i, j) << " ";
        }
        std::cout << "\n";
    }

    // 创建可相乘的矩阵（2x3 和 3x2）演示乘法
    Matrix M1(2, 3);
    M1.set(0,0,1); M1.set(0,1,2); M1.set(0,2,3);
    M1.set(1,0,4); M1.set(1,1,5); M1.set(1,2,6);

    Matrix M2(3, 2);
    M2.set(0,0,7); M2.set(0,1,8);
    M2.set(1,0,9); M2.set(1,1,10);
    M2.set(2,0,11); M2.set(2,1,12);

    Matrix mulResult = M1.multiply(M2);
    std::cout << "\nMatrix M1 * M2 result:\n";
    for (int i = 0; i < mulResult.getRows(); ++i) {
        for (int j = 0; j < mulResult.getCols(); ++j) {
            std::cout << mulResult.get(i, j) << " ";
        }
        std::cout << "\n";
    }

    return 0;
}