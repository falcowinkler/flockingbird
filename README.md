### flockingbird
c++ library for flocking simulations.
http://www.cs.toronto.edu/~dt/siggraph97-course/cwr87/

<img alt="500 boids" src="https://user-images.githubusercontent.com/8613031/119992710-1cf4be80-bfcb-11eb-9ade-cb75ef882987.gif" width="512">

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
### build and test
```bash
./test.sh
```

### emacs integration

make sure that compile_commands is linked to the root

```bash
ln build/compile_commands.json .
```

### Dependencies
For the demo animation:
```bash
brew install cairo
brew install gtk+3
```

NOTE: i had to 
```bash
export LDFLAGS="-L/usr/local/Cellar/cairo/1.16.0_5/lib $LDFLAGS"
```
