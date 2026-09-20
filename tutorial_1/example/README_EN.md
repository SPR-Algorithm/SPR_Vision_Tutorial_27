# Stage 1 Accompanying Examples

Two examples covering all of Stage 1's content:

| Example                  | Directory | Corresponding curriculum section              |
| -------------------------- | --------- | ------------------------------------------------ |
| g++ compilation           | `gpp/`    | 3. g++ and the C/C++ compilation pipeline        |
| Minimal CMake build       | `cmake/`  | 4. CMake: how it works and a minimal example     |

Both examples contain the same vector library code — the only difference is **how it's compiled**: one uses hand-written g++ commands, the other hands the description off to CMake. This lets learners clearly see "what CMake actually does for you."

```
example/
├── gpp/
│   ├── hello.cpp                     ① Single file: see the products of the four compilation steps
│   └── multi/
│       ├── include/vector2d.hpp      ② Multi-file: header holds declarations
│       └── src/vector2d.cpp          ② Multi-file: source holds the implementation
│           src/main.cpp              ② Multi-file: the caller
└── cmake/
    ├── CMakeLists.txt                Minimal CMake build
    ├── include/vector2d.hpp
    └── src/vector2d.cpp
        src/main.cpp
```

---

## ① g++ single file — see the four compilation steps

```bash
cd example/gpp

g++ -std=c++17 -Wall -Wextra hello.cpp -o hello
./hello                              # Hello, SPR Vision!

# Break it into four steps, and see what file each step produces
g++ -E hello.cpp -o hello.i          # Preprocessing: headers get expanded in
g++ -S hello.i   -o hello.s          # Compiling: C++ → assembly
g++ -c hello.s   -o hello.o          # Assembling: assembly → machine-code object file
g++ hello.o      -o hello            # Linking: object file + standard library → executable

ls -lh hello.cpp hello.i hello.s hello.o hello
```

## ② g++ multi-file — see headers and linking

```bash
cd example/gpp/multi

# Correct: both .cpp files must be given; -Iinclude points to the header directory
g++ -std=c++17 -Wall -Wextra -Iinclude src/main.cpp src/vector2d.cpp -o app
./app
```

Output:

```
|a|        = 5
dist(a,b)  = 5
a·b        = 0
2a         = (6, 8)
```

**Negative demo 1: link error** (omit `src/vector2d.cpp`)

```bash
g++ -std=c++17 -Iinclude src/main.cpp -o app_broken
# Linux : undefined reference to `rm::length(rm::Vec2 const&)'
# macOS : Undefined symbols for architecture arm64
# → The declaration was found (the header exists), but the implementation didn't take part in compilation
```

**Negative demo 2: compile error** (comment out `#include "vector2d.hpp"` in `src/main.cpp` and compile live)

```bash
# Linux : error: 'rm' has not been declared
# macOS : error: use of undeclared identifier 'rm'
# → Even the declaration wasn't seen, so linking never even gets a chance
```

## ③ Minimal CMake build

```bash
cd example/cmake

cmake -S . -B build                  # Configure: generate build files
cmake --build build                  # Build: actually compile
./build/vec2_demo
```

Output is the same as ②.

**Bonus demo: incremental build** (touch only one file, and see make only recompile that one)

```bash
touch src/vector2d.cpp
cmake --build build                  # Only vector2d.cpp gets recompiled
```

**Bonus demo: change `PUBLIC` to `PRIVATE`**

Change `target_include_directories(vec2 PUBLIC ...)` to `PRIVATE` in `CMakeLists.txt`, then rebuild:

```
fatal error: 'vector2d.hpp' file not found
```

Reason: `PRIVATE` doesn't propagate the include directory to other targets that link `vec2`, so the `#include "vector2d.hpp"` in `vec2_demo` can no longer find it.

---

## Environment Check

```bash
g++ --version        # Ubuntu: sudo apt install -y build-essential
cmake --version      # Requires >= 3.16, Ubuntu: sudo apt install -y cmake
```

## Platform Notes

- **Ubuntu 22.04**: `g++` is GNU GCC itself.
- **macOS**: `g++` is actually an alias for `clang++`, so error text will differ (e.g. link errors show `Undefined symbols for architecture ...` instead of `undefined reference to ...`), but **the four-step pipeline and all compilation flags are exactly the same**. The dynamic library extension is `.dylib` on macOS and `.so` on Linux.

## Note

`cmake/build/` is the build output directory — do not commit it to git. The `.gitignore` one level up in the repository should include:

```
build/
*.o
*.a
```
