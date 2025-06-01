#!/bin/bash
# Quick fix script for CMake version issues in lbx_robotics

echo "Fixing CMake version issues in lbx_robotics..."

# Fix all CMakeLists.txt and template files
find src -type f \( -name "CMakeLists.txt" -o -name "*.cmake" -o -name "*.cmake.in" -o -name "CMakeLists.txt.in" \) 2>/dev/null | while read -r file; do
    if grep -E "cmake_minimum_required.*VERSION.*[0-3]\.[0-9]" "$file" > /dev/null 2>&1; then
        echo "Fixing: $file"
        # Use perl if available, otherwise sed
        if command -v perl &> /dev/null; then
            perl -i -pe 's/cmake_minimum_required\s*\(\s*VERSION\s+[0-9]+\.[0-9]+(?:\.[0-9]+)?\s*\)/cmake_minimum_required(VERSION 3.11)/gi' "$file"
        else
            sed -i -E 's/cmake_minimum_required[[:space:]]*\([[:space:]]*VERSION[[:space:]]+[0-9]+\.[0-9]+(\.[0-9]+)?[[:space:]]*\)/cmake_minimum_required(VERSION 3.11)/g' "$file"
        fi
    fi
done

# Specifically fix libfranka's GoogleTest template
GOOGLETEST_FILE="src/libfranka/cmake/GoogleTest-CMakeLists.txt.in"
if [ -f "$GOOGLETEST_FILE" ]; then
    echo "Fixing GoogleTest template: $GOOGLETEST_FILE"
    sed -i 's/cmake_minimum_required(VERSION 3.0)/cmake_minimum_required(VERSION 3.11)/g' "$GOOGLETEST_FILE"
fi

# Fix libfranka common submodule
if [ -d "src/libfranka" ]; then
    echo "Checking libfranka submodules..."
    (cd src/libfranka && git submodule update --init --recursive 2>/dev/null || true)
    
    if [ -f "src/libfranka/common/CMakeLists.txt" ]; then
        echo "Fixing libfranka/common/CMakeLists.txt"
        sed -i 's/cmake_minimum_required(VERSION [0-9.]*)/cmake_minimum_required(VERSION 3.11)/g' "src/libfranka/common/CMakeLists.txt"
    fi
fi

# Clean build directory to ensure no cached files with old versions
if [ -d "build" ]; then
    echo "Cleaning build directory to remove cached CMake files..."
    rm -rf build/libfranka 2>/dev/null || true
fi

echo "CMake version fixes completed!"
echo ""
echo "Now you can run: ./unified_launch.sh --clean-build" 