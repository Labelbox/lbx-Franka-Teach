#!/bin/bash
# Troubleshooting script for colcon installation

echo "=== Colcon Installation Troubleshooting ==="
echo ""

echo "1. Checking system information:"
echo "   OS: $(lsb_release -d | cut -f2)"
echo "   Python: $(python3 --version)"
echo "   User: $USER"
echo "   Home: $HOME"
echo ""

echo "2. Checking PATH:"
echo "   Current PATH contains:"
echo "$PATH" | tr ':' '\n' | grep -E "(local|\.local)" || echo "   ❌ No .local directories in PATH"
echo ""

echo "3. Checking ~/.local/bin directory:"
if [ -d "$HOME/.local/bin" ]; then
    echo "   ✓ ~/.local/bin exists"
    echo "   Contents related to colcon:"
    ls -la "$HOME/.local/bin" | grep colcon || echo "   ❌ No colcon executables found"
else
    echo "   ❌ ~/.local/bin does not exist"
fi
echo ""

echo "4. Checking pip installations:"
echo "   Colcon packages installed via pip:"
python3 -m pip list 2>/dev/null | grep colcon || echo "   ❌ No colcon packages found"
echo ""

echo "5. Checking which colcon:"
if command -v colcon &> /dev/null; then
    echo "   ✓ colcon found at: $(which colcon)"
    echo "   Version: $(colcon version-check 2>&1 | grep 'colcon-core' | head -1)"
else
    echo "   ❌ colcon command not found"
fi
echo ""

echo "6. Checking .bashrc for PATH exports:"
if grep -q "export PATH.*\.local/bin" ~/.bashrc; then
    echo "   ✓ PATH export found in .bashrc:"
    grep "export PATH.*\.local/bin" ~/.bashrc
else
    echo "   ❌ No PATH export for .local/bin in .bashrc"
fi
echo ""

echo "=== SOLUTIONS ==="
echo ""

if ! command -v colcon &> /dev/null; then
    echo "To fix colcon not being found:"
    echo ""
    
    if ! echo "$PATH" | grep -q "$HOME/.local/bin"; then
        echo "1. Add ~/.local/bin to PATH for this session:"
        echo "   export PATH=\"\$HOME/.local/bin:\$PATH\""
        echo ""
    fi
    
    if ! python3 -m pip list 2>/dev/null | grep -q colcon; then
        echo "2. Install colcon via pip:"
        echo "   python3 -m pip install --user colcon-common-extensions"
        echo ""
    fi
    
    echo "3. For permanent fix, add to ~/.bashrc:"
    echo "   echo 'export PATH=\"\$HOME/.local/bin:\$PATH\"' >> ~/.bashrc"
    echo "   source ~/.bashrc"
    echo ""
fi

echo "4. If still having issues, try reinstalling:"
echo "   python3 -m pip uninstall -y colcon-common-extensions colcon-core colcon-ros"
echo "   python3 -m pip install --user --upgrade colcon-common-extensions" 