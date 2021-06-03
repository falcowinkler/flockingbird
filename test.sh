cd build
cmake  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=DEBUG ..
make
make install
./flockingbird_tests
cd ..
