#!/bin/bash
# Kill all oculus server processes

echo "🛑 Killing all oculus server processes..."

# Kill all oculus_vr_server processes
pkill -f "oculus_vr_server"

# Also kill any oculus_reader processes if needed
pkill -f "oculus_reader"

# Wait a moment
sleep 0.5

# Check if any processes are still running
if ps aux | grep -E "(oculus_vr_server|oculus_reader)" | grep -v grep > /dev/null; then
    echo "⚠️  Some processes may still be running. Trying with SIGKILL..."
    pkill -9 -f "oculus_vr_server"
    pkill -9 -f "oculus_reader"
else
    echo "✅ All oculus server processes killed successfully"
fi

# Final check
sleep 0.5
if ps aux | grep -E "(oculus_vr_server|oculus_reader)" | grep -v grep > /dev/null; then
    echo "❌ Warning: Some processes are still running:"
    ps aux | grep -E "(oculus_vr_server|oculus_reader)" | grep -v grep
else
    echo "✅ Confirmed: No oculus server processes running"
fi 