cd build
cmake  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
make
make install
./flockingbird_tests
cd ..
