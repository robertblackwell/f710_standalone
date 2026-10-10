#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/rp4-build"

echo "=== Building F710 Project for Raspberry Pi 4 (Debian 12 Bookworm) with Zig ==="

mkdir -p "${BUILD_DIR}"

cmake -B "${BUILD_DIR}" -S "${PROJECT_ROOT}" \
    -DCMAKE_TOOLCHAIN_FILE="${PROJECT_ROOT}/rp4-zig-toolchain.cmake" \
    -DCMAKE_BUILD_TYPE=Release

cmake --build "${BUILD_DIR}" -j"$(nproc)"

echo "=== Build Complete ==="
echo "Binaries located in ${BUILD_DIR}/test_apps/"
