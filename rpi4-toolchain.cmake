# 
# Toolchain file for raspberry pi 4.
# 
# Using gcc this is a set of names like 'aarch-linux-gnu-'
# 
# I have installed these with sudo apt install gcc-12-aarch64-linux-gnu.
# 
# As a result they are located in `/bin` and the associated libraries are in
# '/usr/lib'
# 
# set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch4)

message("rp4-toolchain.cmake - i am here")

set(tools /bin)
set(rootfs_dir $ENV{HOME}/rp4-sysroot)
set(CMAKE_SYSROOT ${rootfs_dir})
set(CMAKE_FIND_ROOT_PATH ${rootfs_dir})

set(CMAKE_C_COMPILER /bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /bin/aarch64-linux-gnu-g++)
set(CMAKE_CXX_ABI_COMPILED  Off)
set(CMAKE_C_ABI_COMPILED  Off)
message(c compiler ${CMAKE_C_COMPILER})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

message("toolchain ended")