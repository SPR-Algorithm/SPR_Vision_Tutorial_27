//   g++ -std=c++17 -Wall -Wextra hello.cpp -o hello && ./hello
//
//   g++ -E hello.cpp -o hello.i    # ① 预处理：展开 #include / #define
//   g++ -S hello.i   -o hello.s    # ② 编译：C++ → 汇编
//   g++ -c hello.s   -o hello.o    # ③ 汇编：汇编 → 机器码目标文件
//   g++ hello.o      -o hello      # ④ 链接：目标文件 + 标准库 → 可执行文件
//
// 输出：Hello, SPR Vision!

#include <iostream>

int main() {
  std::cout << "Hello, SPR Vision!\n";
  return 0;
}
