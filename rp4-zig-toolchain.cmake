# Toolchain file for Raspberry Pi 4 running Debian 12 (Bookworm) using Zig
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_SIZEOF_VOID_P 8)
set(CMAKE_CROSSCOMPILING TRUE)

find_program(CMAKE_MAKE_PROGRAM NAMES make gmake)

# Compiler wrappers targeting aarch64-linux-gnu.2.36
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
set(CMAKE_C_COMPILER "${PROJECT_ROOT}/bin/zig-cc.sh")
set(CMAKE_CXX_COMPILER "${PROJECT_ROOT}/bin/zig-cxx.sh")
set(CMAKE_ASM_COMPILER "${PROJECT_ROOT}/bin/zig-cc.sh")

set(CMAKE_AR "${PROJECT_ROOT}/bin/zig-ar.sh")
set(CMAKE_RANLIB "${PROJECT_ROOT}/bin/zig-ranlib.sh")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
