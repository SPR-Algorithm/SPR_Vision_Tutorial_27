mkdir build
cmake -S . -B build
cmake --build build
cd build
./main
./example
cd ..
rm -R build

