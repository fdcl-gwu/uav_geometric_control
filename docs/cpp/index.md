# FDCL Controllers in C++

## Compiling
Following instructions assume you have git already installed in your system.

1. Install dependencies.
```sh
# Linux
sudo apt-get install -y cmake pkg-config

# Mac
brew install cmake pkg-config
```
2. Update submodules
```sh
git submodule update --init --recursive
```
3. Setup compile directory inside the `cpp` directory.
```sh
mkdir build
cd build
cmake ../
```
4. Compile and run.
```sh
make example
./example
```

For more information, make sure to check the `Classes` tab.

## Unit Tests

1. Build the unit tests first:
```sh
make test_all
```
2. Run the unit test
```sh
./test_all
```
All the tests must pass.

## Generating the Documentation

This C++ code is documented using Doxygen.
Doxygen automatically generates the documentation based on the comments in header files.

If you want to update the documentation, follow the below steps.
1. If not already installed, install `doxygen`:
```sh
# Linux
sudo apt-get install -y doxygen graphviz

# Mac
brew install graphviz doxygen
```
1. Run doxygen (from `master` branch)
```sh
cd Doxygen
doxygen Doxyfile
```