#!/usr/bin/env bash
# Exit immediately if a command exits with a non-zero status
set -e

BOOST_TAG="boost-1.92.0"
EXTERNAL_DIR="external"
BOOST_DIR="${EXTERNAL_DIR}/boost"

echo "=== 1. Cleaning up existing external directories ==="
if [ -d "$EXTERNAL_DIR" ]; then
    echo "Removing old $EXTERNAL_DIR folder..."
    # Deinitializing from git prevents git tracking cache issues
    git submodule deinit -f "$BOOST_DIR" 2>/dev/null || true
    rm -rf ".git/modules/${BOOST_DIR}" 2>/dev/null || true
    rm -rf "$EXTERNAL_DIR"
fi

echo "=== 2. Re-creating external directory structures ==="
mkdir -p "$EXTERNAL_DIR"

echo "=== 3. Adding and checking out Boost Superproject ==="
git submodule add git@github.com:boostorg/boost.git "$BOOST_DIR"
cd "$BOOST_DIR"
git checkout "$BOOST_TAG"

echo "=== 4. Initializing required core submodules ==="
# We must pull tools/boostdep first so the python script exists
git submodule update --init --recursive tools/boostdep
git submodule update --init --recursive libs/asio libs/fiber libs/context

echo "=== 5. Running dependency resolution script ==="
# This downloads all secondary header dependencies required by Asio and Fiber
python3 tools/boostdep/depinst/depinst.py asio
python3 tools/boostdep/depinst/depinst.py fiber

echo "====================================================="
echo " SUCCESS: Boost Asio & Fiber are localized!"
echo " Remember to reload your CMake project in CLion."
echo "====================================================="
