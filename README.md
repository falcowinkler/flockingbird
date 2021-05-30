### flockingbird
A header-only c++ library for creating 2D flocking animations.
http://www.cs.toronto.edu/~dt/siggraph97-course/cwr87/

![demo500](https://user-images.githubusercontent.com/8613031/119993840-52e67280-bfcc-11eb-8697-ae98e67c4900.gif)

The algorithm uses [kD-Trees](https://github.com/jlblancoc/nanoflann) to calculate the neighbors of a boid, making it possible to simulate up to a thousand boids  in decent framerates.

### setup
(I don't have a windows PC, so just for mac)
```bash
brew install cmake
brew install llvm
brew install clang-format # for working on the project/formatting
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

### run demo
```bash
./build/flockingbird_demo
```

### emacs integration

make sure that compile_commands is linked to the root

```bash
ln build/compile_commands.json .
```

### Credits/Disclaimer
Resources used for development
- http://www.vergenet.net/~conrad/boids/pseudocode.html
- https://p5js.org/examples/simulate-flocking.html

### Bindings
- [Swift](https://github.com/falcowinkler/flockingbird-swift) (Work in progress)
