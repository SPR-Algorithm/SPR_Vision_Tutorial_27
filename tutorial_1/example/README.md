# 第一阶段配套实例

两个例子，覆盖第一阶段全部内容：

| 示例           | 目录     | 对应教案                      |
| -------------- | -------- | ----------------------------- |
| g++ 编译       | `gpp/`   | 三、g++ 与 C/C++ 编译流程     |
| CMake 最小构建 | `cmake/` | 四、CMake：作用原理与最小实例 |

两个例子里是同一份向量库代码，区别只在**编译方式**：一个手写 g++ 命令，一个交给 CMake 描述。这样学员能清楚看到「CMake 到底帮你做了什么」。

```
example/
├── gpp/
│   ├── hello.cpp                     ① 单文件：看编译四步的产物
│   └── multi/
│       ├── include/vector2d.hpp      ② 多文件：头文件放声明
│       └── src/vector2d.cpp          ② 多文件：源文件放实现
│           src/main.cpp              ② 多文件：调用方
└── cmake/
    ├── CMakeLists.txt                CMake 最小构建
    ├── include/vector2d.hpp
    └── src/vector2d.cpp
        src/main.cpp
```

---

## ① g++ 单文件 —— 看编译四步

```bash
cd example/gpp

g++ -std=c++17 -Wall -Wextra hello.cpp -o hello
./hello                              # Hello, SPR Vision!

# 拆开四步，看每一步产出什么文件
g++ -E hello.cpp -o hello.i          # 预处理：头文件被展开进来
g++ -S hello.i   -o hello.s          # 编译：C++ → 汇编
g++ -c hello.s   -o hello.o          # 汇编：汇编 → 机器码目标文件
g++ hello.o      -o hello            # 链接：目标文件 + 标准库 → 可执行文件

ls -lh hello.cpp hello.i hello.s hello.o hello
```

## ② g++ 多文件 —— 看头文件与链接

```bash
cd example/gpp/multi

# 正确：两个 .cpp 都要给，-Iinclude 指定头文件目录
g++ -std=c++17 -Wall -Wextra -Iinclude src/main.cpp src/vector2d.cpp -o app
./app
```

输出：

```
|a|        = 5
dist(a,b)  = 5
a·b        = 0
2a         = (6, 8)
```

**反面演示一：链接错误**（漏掉 `src/vector2d.cpp`）

```bash
g++ -std=c++17 -Iinclude src/main.cpp -o app_broken
# Linux : undefined reference to `rm::length(rm::Vec2 const&)'
# macOS : Undefined symbols for architecture arm64
# → 声明找到了（头文件在），实现没参与编译
```

**反面演示二：编译错误**（现场把 `src/main.cpp` 里的 `#include "vector2d.hpp"` 注释掉再编译）

```bash
# Linux : error: 'rm' has not been declared
# macOS : error: use of undeclared identifier 'rm'
# → 连声明都没看到，根本轮不到链接
```

## ③ CMake 最小构建

```bash
cd example/cmake

cmake -S . -B build                  # 配置：生成构建文件
cmake --build build                  # 构建：真正编译
./build/vec2_demo
```

输出同 ②。

**加分演示：增量编译**（只碰一个文件，看 make 只重编它）

```bash
touch src/vector2d.cpp
cmake --build build                  # 只有 vector2d.cpp 被重新编译
```

**加分演示：`PUBLIC` 改成 `PRIVATE`**

把 `CMakeLists.txt` 里的 `target_include_directories(vec2 PUBLIC ...)` 改成 `PRIVATE`，重新构建：

```
fatal error: 'vector2d.hpp' file not found
```

原因：`PRIVATE` 不把 include 目录传给链接 `vec2` 的其他目标，`vec2_demo` 里的 `#include "vector2d.hpp"` 就找不到了。

---

## 环境确认

```bash
g++ --version        # Ubuntu: sudo apt install -y build-essential
cmake --version      # 需要 >= 3.16，Ubuntu: sudo apt install -y cmake
```

## 平台说明

- **Ubuntu 22.04**：`g++` 就是 GNU GCC。
- **macOS**：`g++` 实际是 `clang++` 的别名，报错文字会不同（例如链接错误显示 `Undefined symbols for architecture ...` 而不是 `undefined reference to ...`），但**四步流程和所有编译参数完全一致**。动态库后缀 macOS 是 `.dylib`，Linux 是 `.so`。

## 注意

`cmake/build/` 是构建产物目录，不要提交到 git。往上一层仓库的 `.gitignore` 里要有：

```
build/
*.o
*.a
```
