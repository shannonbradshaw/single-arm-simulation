# single-arm-simulation

Gazebo Harmonic simulation of UFactory xArm 6 with TCP protocol emulator and wrist-mounted Intel RealSense D435 RGBD camera. Controlled via standard Viam SDK through viam-server running inside a GPU Docker container on GCP.

Read `ARCHITECTURE.md` for full architecture diagrams, build/deploy instructions, troubleshooting, and file reference.

## GCP Infrastructure

- VM: xarm6-sim, us-central1-a, n1-standard-8, T4 GPU
- IP: 34.30.31.198
- Firewall: allow-xarm6-web (tcp:8081), allow-xarm6-protocol (tcp:502), allow-xarm6-viam (tcp:8080)
- Network tag: xarm6-web
- Docker image: xarm6-sim-gpu:latest
- Build context on VM: /home/shannon.bradshaw/single-arm-simulation
- Container name: xarm6-full

## Key Architecture Decisions

- Python for emulator (gz-transport Python bindings work well)
- PID force mode for J1-J3 (shoulder/elbow), velocity command mode for J4-J6 (wrist) — DART FORCE actuator fails on wrist joints causing ~28° tracking errors
- Collision bitmasks: arm=0x00, table=0x02, ground=0x02 (arm has NO collisions — self-collision caused 27-31° tracking errors on wrist joints)
- Physics timestep: 2ms (0.002), 500Hz update rate — needed for PID stability
- Gripper finger joints: all fixed (not revolute) — eliminates jitter from unactuated DOFs
- Safe home position: [0, -45, -30, 0, 60, 0] degrees — canonical starting pose, gripper above table. Always use this as the initial arm pose.

## Why No Arm-Table Collision

At all-zeros, the xArm6 TCP is 6cm BELOW the base (gripper through table). Gazebo Harmonic's `<initial_position>` doesn't reliably set spawn angles — arm spawns at zero, gets stuck in table collision. Arm bitmask stays 0x01 (no table collision). Safe home position keeps gripper above table in practice.

## Critical Bugs Found & Fixed

1. **Link2-table collision**: link2's STL mesh extends below shoulder. Fixed with bitmask separation.
2. **GetState polling timeout**: Fixed with time-based approach (return "idle" after 3s settle time).
3. **Jitter**: High P gains + low damping caused visible oscillation. Fixed by reducing P gains, increasing D gains, increasing joint damping (15-30 Nm·s/rad), fixing gripper finger joints, reducing timestep to 2ms.
4. **Gripper-in-table at home**: TCP at -6cm below base at all-zeros. Fixed with safe home [0,-45,-30,0,60,0]°.

## Joint Controller Configuration (current working values)

All controllers use `i_max`/`i_min` to prevent integral windup (gz-sim #1684).
Collision bitmask on all arm links is 0x00 (no collisions) to prevent self-collision forces.
Invalid `<initial_position>` inside `<axis>` elements MUST NOT be used — Gazebo copies the unrecognized element, causing unpredictable joint behavior.

- J1: P=300, I=5, D=100, cmd=±100, damping=20 (PID force mode)
- J2: P=600, I=10, D=200, cmd=±150, damping=30 (PID force mode, gravity load)
- J3: P=400, I=10, D=120, cmd=±150, damping=20 (PID force mode)
- J4: velocity commands mode (use_velocity_commands=true), cmd=±3.14 rad/s, damping=8. PID force mode has DART actuator issues causing ~28° offsets in multi-joint poses.
- J5: velocity commands mode (use_velocity_commands=true), cmd=±3.14 rad/s, damping=15. PID force mode does NOT work — DART FORCE actuator fails to apply sufficient torque.
- J6: velocity commands mode (use_velocity_commands=true), cmd=±3.14 rad/s, damping=3. PID force mode has DART actuator issues causing ~28° offsets in multi-joint poses.
- Gripper drive: P=50, I=1, D=20, cmd=±10, damping=10

## Protocol Details

xArm Private Modbus TCP wire format (both request and response):
`[TransactionID:2 BE][ProtocolID:2 BE=0x0002][Length:2 BE][Register:1][Payload]`

Float payloads are IEEE 754 float32 little-endian. Header integers are big-endian.

Key registers: GET_VERSION(0x01), MOTION_EN(0x0B), SET_STATE(0x0C), GET_STATE(0x0D), MOVE_SERVOJ(0x1D), GRIPPER_SET_POS(0x3E).

## Viam Module Integration

- Module source: github.com/viam-modules/viam-ufactory-xarm (has main.go — local clone at /tmp/viam-ufactory-xarm was incomplete, missing main.go)
- Binary (macOS): /tmp/xarm-module/ufactory-xarm (arm64)
- Binary (Linux): built on VM with Go 1.22, in Docker image at /opt/viam-xarm (~103MB)
- SDK: viam-sdk 0.71.1 in /tmp/viam-test-env venv (local), 0.71.1 in container
- viam-server: v0.115.0, AppImage extracted in container, must invoke with explicit linker
- In-container config: viam_config_container.json (host=127.0.0.1, port 8080)
- Module init sequence: GET_ERROR → MOTION_EN → SET_MODE(1) → SET_STATE(0)
- Motion uses MOVE_SERVOJ (0x1D) with interpolated waypoints, polls GetState (0x0D)
- Config attrs: speed_degs_per_sec=100, acceleration=200, move_hz=20

## D435 Wrist Camera (end-to-end verified)

- RGBD sensor on d435_link, visual mesh on link6 (link_eef frame)
- Mount transform from xarm_ros2: xyz=0.06746 -0.0175 0.0237, rpy=π -π/2 0
- gz_camera_module/: Python Viam module using gz-transport subscriptions
- Protobuf compat: removed protobuf==3.20.3 pin, set PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
- GetImages patch: viam-server Go needs proto Format enum; patched CameraRPCService.GetImages in main.py
- Returns JPEG color (24-28KB) + VIAM_DPTH depth (1.84MB) at 1280x720, 15Hz sensor rate

## Build artifacts NOT in git

These live only in the VM build context at ~/single-arm-simulation and are COPY'd by Dockerfile.gpu:
- `viam-xarm-linux` (~103MB) — build with: `git clone github.com/viam-modules/viam-ufactory-xarm && go build -o viam-xarm-linux .`
- `viam-server-appimage` (~40MB) — download Linux amd64 AppImage from Viam releases
