#!/bin/bash
# Start Gazebo, xArm protocol emulator, and web viewer.
set -e

echo "=== Starting Gazebo Harmonic ==="
gz sim -s -r --headless-rendering /opt/worlds/xarm6_table.sdf &
GZ_PID=$!

echo "Waiting for Gazebo to initialize..."
sleep 10

echo "=== Starting xArm protocol emulator (port 502) ==="
python3 /opt/xarm_emulator.py &
EMU_PID=$!

echo "=== Starting web viewer (port 8081) ==="
python3 /opt/web_viewer.py &
WEB_PID=$!

echo "All services running: gz=$GZ_PID emu=$EMU_PID web=$WEB_PID"

# Wait for any child to exit, then shut down all
wait -n
echo "A service exited, shutting down..."
kill $GZ_PID $EMU_PID $WEB_PID 2>/dev/null
wait
