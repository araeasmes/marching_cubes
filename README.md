Attempt at generating marching cubes lookup tables

Based on raylib CMake Project template as well as raylib examples
Uses raylib fork with uint32 mesh indices - https://github.com/araeasmes/raylib\_uint32\_indices

Includes a copy of glm lib, there are plans to modify glm to be compatible with raylib's vector and matrix types
Assumes raylib is installed, headers are provided with the project

Suggested way to build is to use "cmake .." from ./build directory, followed by make. The resources folder is supposed to be in the same directory as the executable.

