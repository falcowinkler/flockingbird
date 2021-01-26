### flockingbird
c++ library for flocking simulations.
http://www.cs.toronto.edu/~dt/siggraph97-course/cwr87/


### prerequisites
```bash
brew install cmake
brew install llvm
brew install clang-format
```

### build

```bash
cd build
cmake ..
make && make install
```

### test (after build)
```bash
./build/flockingbird_tests
```

### run test simulations
```bash
./build/flockingbird
```

### emacs integration

make sure that compile_commands is linked to the root

```bash
ln build/compile_commands.json .
```

