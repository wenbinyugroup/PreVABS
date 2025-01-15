@echo off

rd /s /q build_msvc
mkdir build_msvc
cd build_msvc
cmake ^
    -D CMAKE_PREFIX_PATH="../../.." ^
    ..

cmake --build . --config Release
