mkdir build
cmake -S . -B build
cmake --build build
cd build/src/
./test
cd ../..
rm -R build
