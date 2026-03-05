# xArm 6 Gazebo Simulation — Architecture & Developer Guide

Gazebo Harmonic simulation of a UFactory xArm 6 with xArm Gripper and a wrist-mounted Intel RealSense D435 RGBD camera. The simulation runs inside a GPU-accelerated Docker container on GCP and exposes the standard Viam SDK interface — any Viam client can control the arm and capture camera images by connecting to port 8080.

## Table of Contents

- [Quick Start](#quick-start)
- [Architecture Overview](#architecture-overview)
- [GCP Infrastructure](#gcp-infrastructure)
- [Docker Container](#docker-container)
- [Simulation Components](#simulation-components)
- [Viam Integration](#viam-integration)
- [Key Design Decisions & Gotchas](#key-design-decisions--gotchas)
- [File Reference](#file-reference)
- [Build & Deploy](#build--deploy)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)

---

## Quick Start

### Connect from any machine with the Viam Python SDK

```python
import asyncio
from viam.robot.client import RobotClient
from viam.rpc.dial import DialOptions
from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.proto.component.arm import JointPositions

async def main():
    opts = RobotClient.Options(dial_options=DialOptions(insecure=True))
    robot = await RobotClient.at_address("34.30.31.198:8080", opts)

    arm = Arm.from_robot(robot, "xarm6")
    cam = Camera.from_robot(robot, "wrist-cam")

    # Move to a pose (6 joint angles in degrees)
    await arm.move_to_joint_positions(JointPositions(values=[0, -45, -30, 0, 60, 0]))

    # Capture color + depth images
    images, metadata = await cam.get_images()
    for img in images:
        print(f"{img.name}: {len(img.data)} bytes, {img.mime_type}")

    await robot.close()

asyncio.run(main())
```

### Ports

| Port | Service | Protocol |
|------|---------|----------|
| 502  | xArm TCP protocol emulator | Modbus TCP (Private) |
| 8080 | viam-server (gRPC) | Viam SDK |
| 8081 | Web viewer (Flask) | HTTP |

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│  GCP VM: xarm6-sim (n1-standard-8 + T4 GPU)                        │
│                                                                     │
│  ┌───────────────────── Docker Container ──────────────────────┐    │
│  │                                                             │    │
│  │  ┌─────────────┐    gz-transport     ┌──────────────────┐  │    │
│  │  │   Gazebo     │◄──────────────────►│  xarm_emulator   │  │    │
│  │  │  Harmonic    │  joint cmds/state   │  (port 502)      │  │    │
│  │  │  (physics +  │                     └────────┬─────────┘  │    │
│  │  │   rendering) │                              │             │    │
│  │  │             │                     xArm TCP protocol       │    │
│  │  │  rgbd_camera │                              │             │    │
│  │  │  sensor      │                              │             │    │
│  │  └──────┬───────┘                    ┌─────────▼──────────┐  │    │
│  │         │                            │   viam-server      │  │    │
│  │    gz-transport                      │   (port 8080)      │  │    │
│  │    /d435/image                       │                    │  │    │
│  │    /d435/depth_image                 │  ┌──────────────┐  │  │    │
│  │         │                            │  │ viam-xarm    │  │  │    │
│  │  ┌──────▼───────┐                   │  │ module       │  │  │    │
│  │  │ gz_camera    │◄──────────────────┤  │ (arm)        │  │  │    │
│  │  │ module       │  Viam Camera API   │  └──────────────┘  │  │    │
│  │  │ (camera)     │──────────────────►│                    │  │    │
│  │  └──────────────┘                   └────────────────────┘  │    │
│  │                                                             │    │
│  │  ┌──────────────┐                                           │    │
│  │  │ web_viewer   │  Flask (port 8081)                        │    │
│  │  └──────────────┘                                           │    │
│  └─────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────┘
         ▲                           ▲
         │ HTTP :8081                │ gRPC :8080
         │ (browser)                 │ (Viam SDK)
         ▼                           ▼
    ┌──────────┐              ┌──────────────┐
    │  Browser  │              │  Viam Client  │
    └──────────┘              └──────────────┘
```

### Data Flow

1. **Viam client** sends `MoveToJointPositions` via gRPC to **viam-server** (port 8080)
2. **viam-server** delegates to the **viam-xarm module**, which speaks xArm TCP protocol to port 502
3. **xarm_emulator** receives the command, publishes target joint positions via **gz-transport** to Gazebo's PID controllers
4. **Gazebo** simulates the physics, moves the arm, renders the scene
5. The **rgbd_camera sensor** on the wrist publishes color and depth images via gz-transport
6. **gz_camera_module** subscribes to those topics and serves them through the Viam Camera API
7. **Viam client** calls `GetImages` and receives JPEG color + Viam depth data

---

## GCP Infrastructure

| Resource | Value |
|----------|-------|
| VM name | `xarm6-sim` |
| Zone | `us-central1-a` |
| Machine type | `n1-standard-8` |
| GPU | NVIDIA T4 |
| External IP | `34.30.31.198` |
| Firewall rules | `allow-xarm6-web` (tcp:8081), `allow-xarm6-protocol` (tcp:502), `allow-xarm6-viam` (tcp:8080) |
| Network tag | `xarm6-web` |
| VM provisioning | `startup-script.sh` installs NVIDIA drivers 535, Docker, nvidia-container-toolkit |
| Docker image | `xarm6-sim-gpu:latest` |
| Container name | `xarm6-full` |

### Build context on VM

The Docker build context lives at `/home/shannon.bradshaw/single-arm-simulation` on the VM. It contains two large binaries that are **not in the git repo** (build artifacts):

| File | Size | Source |
|------|------|--------|
| `viam-xarm-linux` | ~103 MB | Built from `github.com/viam-modules/viam-ufactory-xarm` with Go 1.22 on the VM |
| `viam-server-appimage` | ~40 MB | Downloaded from Viam releases (v0.115.0) |

These are `COPY`'d into the Docker image during build. See [Build & Deploy](#build--deploy) for how to recreate them.

---

## Docker Container

### Dockerfile.gpu

Based on `nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04`. Installs:

- Gazebo Harmonic (`gz-harmonic`)
- GPU rendering libs (EGL, Vulkan, Mesa)
- Python 3 with `python3-gz-transport13`, `python3-gz-msgs10`
- pip packages: `flask`, `Pillow`, `numpy`, `viam-sdk`

### Key environment variables

| Variable | Value | Purpose |
|----------|-------|---------|
| `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION` | `python` | **Critical.** gz-msgs10 system package was compiled against protobuf 3.x. viam-sdk requires protobuf 5.x. This env var forces the pure-Python protobuf implementation, which is compatible with both. Without it, the gz-transport Python bindings crash on import. |
| `GZ_SIM_RESOURCE_PATH` | `/opt/models` | Tells Gazebo where to find the xarm6 model |
| `LIBGL_ALWAYS_SOFTWARE` | `0` | Use hardware GPU rendering |
| `__GLX_VENDOR_LIBRARY_NAME` | `nvidia` | Force NVIDIA GL driver |

### AppImage extraction

viam-server is distributed as an AppImage, which normally requires FUSE. Inside Docker there's no FUSE, so the Dockerfile extracts it:

```dockerfile
RUN cd /opt && /opt/viam-server --appimage-extract > /dev/null 2>&1 && \
    ln -sf /opt/squashfs-root/usr/bin/viam-server /opt/viam-server-bin && \
    rm /opt/viam-server
```

The extracted binary has a **relative ELF interpreter** (`lib64/ld-linux-x86-64.so.2`), so it must be invoked with the explicit system linker:

```bash
/lib64/ld-linux-x86-64.so.2 /opt/squashfs-root/usr/bin/viam-server -config /opt/viam_config.json
```

### start.sh — process lifecycle

Launches 4 processes in order, waits for any to exit, then kills all:

1. `gz sim -s -r --headless-rendering` — Gazebo (headless server + GPU rendering)
2. `xarm_emulator.py` — TCP protocol bridge (after 10s delay for Gazebo init)
3. `web_viewer.py` — Flask camera viewer
4. `viam-server` — gRPC server with arm + camera modules

---

## Simulation Components

### xArm 6 Robot Model (`models/xarm6/model.sdf`)

925-line SDF 1.9 model converted from the `xarm_ros2` URDF (jazzy branch). Includes:

- **6 revolute joints** (joint1–joint6) with position controllers (J1–J3: PID force mode, J4–J6: velocity command mode — see [Gotchas](#key-design-decisions--gotchas))
- **xArm Gripper** with drive_joint (all finger linkage joints are **fixed**, not revolute — see [Gotchas](#key-design-decisions--gotchas))
- **D435 camera** on wrist (see below)
- **Collision bitmasks**: all arm links = `0x00` (no collisions at all — see [Gotchas](#key-design-decisions--gotchas)), table/ground = `0x02`

#### Joint Controller Configuration

Tuned for stability with DART physics at 2ms timestep, 500Hz. All PID controllers use `i_max`/`i_min` to prevent integral windup ([gz-sim #1684](https://github.com/gazebosim/gz-sim/issues/1684)).

**J1–J3 use PID force mode** (JointPositionController with PID gains):

| Joint | P | I | D | Cmd Limit | Damping | Notes |
|-------|---|---|---|-----------|---------|-------|
| J1 | 300 | 5 | 100 | ±100 | 20 | Base rotation |
| J2 | 600 | 10 | 200 | ±150 | 30 | Shoulder (highest gravity load) |
| J3 | 400 | 10 | 120 | ±150 | 20 | Elbow |
| Gripper | 50 | 1 | 20 | ±10 | 10 | Drive joint only |

**J4–J6 use velocity command mode** (`<use_velocity_commands>true</use_velocity_commands>`):

| Joint | Cmd Limit (rad/s) | Damping | Notes |
|-------|-------------------|---------|-------|
| J4 | ±3.14 | 8 | Wrist rotation |
| J5 | ±3.14 | 15 | Wrist bend |
| J6 | ±3.14 | 3 | Tool rotation |

**Why velocity mode for wrist joints?** PID force mode uses `JointForceCmd`, which sets DART's actuator type to FORCE. This actuator type fails to apply sufficient torque on the wrist joints (J4–J6), causing ~28° tracking errors in multi-joint poses. The root cause is a DART physics engine limitation. Velocity command mode bypasses this entirely — the controller computes a velocity command that drives the joint to the target position, achieving sub-1° tracking accuracy on all joints. See [Wrist joint tracking](#wrist-joint-tracking-dart-force-actuator-bug) for details.

### Intel RealSense D435 Wrist Camera

Mounted on link6 using UFactory's standard camera stand bracket.

**Mount transform** (from `xarm_ros2` `realsense_d435i.urdf.xacro`):
- Position relative to link6: `xyz="0.06746 -0.0175 0.0237"` (67mm forward, 17.5mm right, 23.7mm up)
- Orientation: `rpy="3.14159 -1.5708 0"` (π, -π/2, 0)

**Visual mesh**: `models/xarm6/meshes/d435/d435_with_cam_stand.stl` — combined camera + bracket, attached to link6 visual (mesh origin = link_eef frame).

**Sensor** (`rgbd_camera` type on `d435_link`):
- Resolution: 1280x720 (color and depth)
- Horizontal FOV: 1.5184 rad (87°, matching D435 depth FOV)
- Depth range: 0.105m – 10.0m
- Update rate: 15 Hz
- Topic prefix: `/d435` → publishes `/d435/image`, `/d435/depth_image`, `/d435/points`
- Noise: Gaussian σ=0.007 on color

### World (`worlds/xarm6_table.sdf`)

- Physics: DART, 2ms timestep, 500Hz, real-time factor 1.0
- Scene: ambient lighting, shadows, ogre2 renderer
- Objects: ground plane, 80×60cm table (75cm height), xArm 6 on table, overhead lighting
- Plugins: Physics, SceneBroadcaster, UserCommands, Sensors (ogre2), Contact

### xArm Protocol Emulator (`xarm_emulator.py`)

Bridges UFactory's Private Modbus TCP protocol to Gazebo gz-transport. The real Viam xArm module connects to it with zero code changes — the only difference vs. a real arm is the IP address.

**Wire format**: `[TransactionID:2 BE][ProtocolID:2 BE=0x0002][Length:2 BE][Register:1][Payload]`

**Supported registers**:

| Register | Name | Function |
|----------|------|----------|
| 0x01 | GET_VERSION | Returns firmware version string |
| 0x0A | SYSTEM_CONTROL | Arm enable/disable |
| 0x0B | MOTION_EN | Motor enable |
| 0x0C | SET_STATE | State machine control |
| 0x0D | GET_STATE | Returns current state + error + joint positions |
| 0x0E | GET_CMDNUM | Command queue length |
| 0x10 | GET_ERROR | Error code |
| 0x1D | MOVE_SERVOJ | Servo joint move command (the main motion command) |
| 0x3A | GRIPPER_SET_EN | Gripper enable |
| 0x3B | GRIPPER_SET_MODE | Gripper mode |
| 0x3C | GRIPPER_SET_ZERO | Set gripper zero position |
| 0x3D | GRIPPER_GET_POS | Get gripper position |
| 0x3E | GRIPPER_SET_POS | Set gripper position |
| 0x3F | GRIPPER_SET_SPEED | Set gripper speed |

**Joint position reporting**: `GET_JOINT_POS` returns the last *commanded* target positions, not actual Gazebo physics positions. A real xArm's servos hold commanded positions near-perfectly; in sim the PID controllers have small steady-state errors under gravity. Returning actual Gazebo positions would cause the ufactory module's interpolation to accumulate error over successive moves.

**State machine**: Uses time-based idle detection — returns state=1 (moving) while joints are changing, state=2 (idle) after 3 seconds of stability. This avoids polling timeouts that occurred with threshold-based detection.

**Startup**: Publishes home position `[0, -45, -30, 0, 60, 0]°` on startup so the arm begins in a safe pose with the gripper above the table.

---

## Viam Integration

### viam-server Configuration

Two config modes are supported, selected via `VIAM_CONFIG` env var in `start.sh`:

- **Local mode** (default): `viam_config_container.json` — standalone, no cloud connection. Components defined locally.
- **Cloud mode**: `viam_config_cloud.json` — connects to app.viam.com. Components and modules are managed through the Viam web UI. Use `docker run -e VIAM_CONFIG=/opt/viam_config_cloud.json ...` to enable.

The local config registers two modules:

**Arm module** (`viam:ufactory:xArm6`):
- Binary: `/opt/viam-xarm` (Go, built from `github.com/viam-modules/viam-ufactory-xarm`)
- Connects to emulator at `127.0.0.1:502`
- Attributes: `speed_degs_per_sec=100`, `acceleration=200`, `move_hz=20`
- Init sequence: `GET_ERROR → MOTION_EN → SET_MODE(1) → SET_STATE(0)`
- Motion: sends `MOVE_SERVOJ` (0x1D) with interpolated waypoints, polls `GetState` (0x0D)

**Camera module** (`viam:camera:gazebo-rgbd`):
- Executable: `/opt/run_camera_module.sh` → `python3 -m gz_camera_module.main <socket_path>`
- Subscribes to gz-transport topics `/d435/image` and `/d435/depth_image`
- Serves two image sources: `color` (JPEG, ~25KB) and `depth` (Viam DPTH format, ~1.8MB)
- Attributes: `topic_prefix="/d435"`, `width=1280`, `height=720`

### Camera Module Implementation (`gz_camera_module/`)

**gz_camera.py** — `GzCamera` class:
- Extends `Camera` and `Reconfigurable`
- Creates a `gz.transport13.Node` and subscribes to color + depth topics
- Callbacks store latest frame data under thread locks
- `get_images()` returns `List[NamedImage]`:
  - Color: raw RGB bytes → PIL → JPEG (quality 90) → `NamedImage("color", bytes, CameraMimeType.JPEG)`
  - Depth: R_FLOAT32 (meters) → Viam depth format (9-byte magic `VIAM_DPTH` + uint64 width + uint64 height + big-endian uint16 pixels in mm) → `NamedImage("depth", bytes, CameraMimeType.VIAM_RAW_DEPTH)`
- `get_properties()` returns intrinsics computed from the 87° HFOV

**main.py** — Module entry point:
- **Monkey-patches `CameraRPCService.GetImages`** to set the proto `Format` enum field. This is necessary because the Python SDK's built-in handler only sets the `mime_type` string field, but viam-server's Go code requires the `format` enum (FORMAT_JPEG=3, etc.) to decode images. Without this patch, viam-server returns `"image: unknown format"` errors.
- Registers `GzCamera` with `Registry.register_resource_creator()` using SDK 0.71.1 API (`Camera.API`, `ResourceCreatorRegistration` wrapper)

### test_config.json

Local development config (not used in the container). Points the arm module at the VM's external IP for testing from a local machine without viam-server in the container.

---

## Key Design Decisions & Gotchas

### No arm collisions (bitmasks)

All arm link collision bitmasks are set to `0x00` (no collisions at all). Table and ground use `0x02`. Two reasons:

1. **Arm-table collision**: At joint angles `[0,0,0,0,0,0]`, the xArm 6 TCP is **6cm below the base** — the gripper goes through the table. Gazebo Harmonic's `<initial_position>` doesn't reliably set spawn angles, so the arm briefly spawns at all-zeros and gets stuck if table collision is enabled.

2. **Self-collision**: With bitmask `0x01`, non-adjacent arm links collide with each other. This creates constraint forces of 100+ Nm that the PID controllers can't overcome, causing 27–31° tracking errors on wrist joints (J4, J5, J6). Setting bitmask to `0x00` eliminates both arm-table and arm-self collisions.

The safe home position `[0,-45,-30,0,60,0]°` keeps the gripper above the table during normal operation.

### Fixed gripper finger joints

All gripper finger linkage joints (except `drive_joint`) are **fixed joints**, not revolute. With revolute joints, the unactuated finger links caused visible jitter because the PID controllers had nothing to stabilize them against. Making them fixed eliminates this entirely.

### Protobuf 3.x / 5.x coexistence

The `python3-gz-msgs10` system package was compiled against protobuf 3.x. The `viam-sdk` requires protobuf 5.x. These are normally incompatible. Setting `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` forces the pure-Python protobuf runtime, which can parse both versions' generated code. This is a performance tradeoff (pure Python is slower than C++ extension) but acceptable for the camera module's throughput.

### GetImages Format enum patch

The Python Viam SDK's `CameraRPCService.GetImages` handler builds `Image` protos with `mime_type` set but `format` left at `FORMAT_UNSPECIFIED` (0). viam-server's Go code dispatches on the `format` enum field, not `mime_type`, so it fails with `"image: unknown format"`. The monkey-patch in `main.py` rebuilds the proto response with the correct enum values.

### AppImage + relative ELF interpreter

viam-server's AppImage extracts a binary with interpreter `lib64/ld-linux-x86-64.so.2` (relative, not `/lib64/...`). This fails inside Docker because the CWD isn't the squashfs-root. Invoking via the system linker explicitly works: `/lib64/ld-linux-x86-64.so.2 /opt/squashfs-root/usr/bin/viam-server`.

### Wrist joint tracking (DART FORCE actuator bug)

J4, J5, and J6 all exhibit the same problem with PID force mode: Gazebo's `JointPositionController` uses `JointForceCmd` to apply torque, which sets DART's actuator type to FORCE. This actuator type bypasses joint friction and fails to apply sufficient torque under dynamic multi-joint loads. Symptoms:

- **J5**: stuck at ~17° instead of target 60° (42° error), regardless of PID gain values
- **J4, J6**: track correctly at home position but drift ~28° when other joints move simultaneously

The fix is `<use_velocity_commands>true</use_velocity_commands>` in the JointPositionController plugin. This mode computes a velocity command rather than a force command, which DART handles correctly. Result: sub-1° tracking on all joints, zero coupling between joints.

**Do NOT use `<initial_position>` inside `<axis>` elements.** This is not valid SDF. Gazebo warns "XML Element[initial_position], child of element[axis], not defined in SDF. Copying[initial_position] as children of [axis]." The unrecognized element causes unpredictable joint behavior. Only use `<initial_position>` inside JointPositionController plugins.

### Physics tuning

- **2ms timestep** (500Hz): Required for PID stability. At 10ms the arm oscillates visibly.
- **High damping** (15–30 Nm·s/rad): Prevents overshoot. J2 has the highest damping (30) because it bears the most gravity load.
- **Command limits**: `i_max`/`i_min` on PID controllers prevent integral windup ([gz-sim #1684](https://github.com/gazebosim/gz-sim/issues/1684)). `cmd_max`/`cmd_min` limit force output (PID mode) or velocity (velocity mode).

### Network latency

The Viam xArm module sends interpolated waypoints via MOVE_SERVOJ at `move_hz=20`. With ~89ms round-trip latency from an external machine, trajectory delivery can't keep up. For best motion quality, the module should run co-located with the simulation (which it does inside the Docker container via viam-server).

---

## File Reference

```
single-arm-simulation/
├── ARCHITECTURE.md              ← This file
├── CLAUDE.md                    ← AI assistant context (key decisions, PID gains, gotchas)
├── Dockerfile                   ← CPU-only image (no GPU, xvfb-based, no viam-server)
├── Dockerfile.gpu               ← GPU image with viam-server + modules (production)
├── start.sh                     ← Container entrypoint: starts all 4 services
├── startup-script.sh            ← GCP VM provisioning (NVIDIA drivers, Docker, nvidia-ctk)
├── xarm_emulator.py             ← xArm TCP protocol ↔ gz-transport bridge
├── web_viewer.py                ← Flask app serving camera feed on port 8081
├── run_camera_module.sh         ← Shell wrapper to launch gz_camera_module
├── viam_config_container.json   ← viam-server config (local mode, both modules)
├── viam_config_cloud.json       ← viam-server config (cloud mode, app.viam.com)
├── test_config.json             ← viam-server config (local dev, external IP)
├── test_arm_joints.py           ← Automated joint movement test suite (SDK + Gazebo verification)
├── quick_diag.py                ← Quick diagnostic: home move + J5 tracking check
├── gz_camera_module/
│   ├── __init__.py
│   ├── main.py                  ← Module entry point + GetImages proto patch
│   └── gz_camera.py             ← GzCamera component (gz-transport → Viam Camera API)
├── models/
│   └── xarm6/
│       ├── model.sdf            ← xArm 6 + gripper + D435 camera (~920 lines)
│       ├── model.config
│       └── meshes/
│           ├── visual/          ← Arm link STL meshes
│           ├── collision/       ← Simplified collision meshes
│           ├── gripper/         ← Gripper link meshes
│           └── d435/
│               └── d435_with_cam_stand.stl  ← Camera + bracket mesh
└── worlds/
    └── xarm6_table.sdf          ← World: table, lighting, physics config
```

### Files NOT in git (VM build context only)

| File | Description | How to recreate |
|------|-------------|-----------------|
| `viam-xarm-linux` | ufactory-xarm module (Linux amd64, ~103MB) | `git clone github.com/viam-modules/viam-ufactory-xarm && cd viam-ufactory-xarm && go build -o viam-xarm-linux` |
| `viam-server-appimage` | viam-server AppImage (~40MB) | Download from Viam releases for Linux amd64 |

---

## Build & Deploy

### Prerequisites on the GCP VM

SSH in: `gcloud compute ssh xarm6-sim --zone=us-central1-a`

The VM should already have Docker + NVIDIA runtime from `startup-script.sh`. The build context is at `~/single-arm-simulation`.

### Build the xarm module binary (if needed)

```bash
# On the VM
cd /tmp
git clone https://github.com/viam-modules/viam-ufactory-xarm.git
cd viam-ufactory-xarm
go build -o ~/single-arm-simulation/viam-xarm-linux .
```

### Get viam-server AppImage (if needed)

Download the latest Linux amd64 AppImage from Viam's releases and place it at `~/single-arm-simulation/viam-server-appimage`.

### Build and run the Docker image

```bash
cd ~/single-arm-simulation
sudo docker build -f Dockerfile.gpu -t xarm6-sim-gpu:latest .

# Stop existing container if running
sudo docker rm -f xarm6-full 2>/dev/null

# Run with GPU access — LOCAL mode (standalone, no cloud)
sudo docker run -d \
  --name xarm6-full \
  --runtime=nvidia \
  --gpus all \
  -p 502:502 \
  -p 8080:8080 \
  -p 8081:8081 \
  xarm6-sim-gpu:latest

# Or CLOUD mode (connects to app.viam.com)
sudo docker run -d \
  --name xarm6-full \
  --runtime=nvidia \
  --gpus all \
  -p 502:502 \
  -p 8080:8080 \
  -p 8081:8081 \
  -e VIAM_CONFIG=/opt/viam_config_cloud.json \
  xarm6-sim-gpu:latest
```

### Sync local changes to VM

```bash
# From your local machine (use --tunnel-through-iap if direct SSH times out)
gcloud compute scp --tunnel-through-iap --recurse \
  --zone=us-central1-a \
  /path/to/single-arm-simulation/ \
  shannon.bradshaw@xarm6-sim:~/single-arm-simulation/
```

(Exclude the large binaries — they should already be on the VM.)

---

## Testing

### Check container health

```bash
# On the VM
sudo docker logs xarm6-full
# Should show: Gazebo started, emulator listening, web viewer running, viam-server started
```

### Verify from a client machine

```bash
pip install viam-sdk Pillow
```

```python
# Minimal connectivity test
import asyncio
from viam.robot.client import RobotClient
from viam.rpc.dial import DialOptions

async def main():
    opts = RobotClient.Options(dial_options=DialOptions(insecure=True))
    robot = await RobotClient.at_address("34.30.31.198:8080", opts)
    print([r.name for r in robot.resource_names])
    await robot.close()

asyncio.run(main())
# Expected output includes: 'xarm6', 'wrist-cam'
```

### Automated joint movement tests

`test_arm_joints.py` exercises all 6 joints through single-joint sweeps, multi-joint poses, and coupling analysis. It connects via the Viam SDK and optionally reads actual Gazebo joint states via gcloud SSH for physics-level verification.

```bash
# Set credentials (from Viam app CONNECT tab, or use local mode with direct IP)
export VIAM_API_KEY="..."
export VIAM_API_KEY_ID="..."
export VIAM_ADDRESS="xarm6-sim-main.XXXXX.viam.cloud"

# Run all tests
python3 test_arm_joints.py --test all

# Or run specific suites
python3 test_arm_joints.py --test single    # Single-joint sweeps (22 moves)
python3 test_arm_joints.py --test multi     # Multi-joint poses (7 moves)
python3 test_arm_joints.py --test coupling  # Coupling analysis (7 moves)
```

All 74 tests pass with the current configuration (velocity mode for J4–J6, bitmask 0x00).

### Quick diagnostic

`quick_diag.py` moves to home and tests J5 at several angles — useful for quickly verifying the simulation is tracking correctly after changes.

### End-to-end arm + camera test

See the [Quick Start](#quick-start) example.

### Web viewer

Open `http://34.30.31.198:8081` in a browser to see the live camera feed.

---

## Troubleshooting

### "image: unknown format" from camera

The `GetImages` proto Format enum patch in `gz_camera_module/main.py` isn't being applied. Make sure `main.py` is the module entry point (not `gz_camera.py` directly) and that the monkey-patch runs before the module starts.

### Camera returns no images

Gazebo's `rgbd_camera` sensor takes a few seconds to start publishing after the simulation loads. The `get_images()` call silently returns an empty list if no frames have arrived yet. Wait for Gazebo to fully initialize (~10s).

### Arm doesn't move / times out

1. Check that the emulator is running: `sudo docker exec xarm6-full ps aux | grep emulator`
2. Check emulator logs for connection: `sudo docker logs xarm6-full 2>&1 | grep -i emulator`
3. The xarm module init sequence (`GET_ERROR → MOTION_EN → SET_MODE → SET_STATE`) must complete. Check viam-server logs for errors.

### Protobuf import errors

If you see `TypeError` or `AttributeError` from protobuf-related code, verify that `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python` is set. This is the #1 cause of mysterious crashes when gz-msgs10 and viam-sdk coexist.

### Gripper passes through table

This is intentional. See [No arm-table collision](#no-arm-table-collision-bitmasks). The safe home position `[0,-45,-30,0,60,0]°` keeps the gripper above the table.

### SSH to VM times out

Use IAP tunneling: `gcloud compute ssh --tunnel-through-iap shannon.bradshaw@xarm6-sim --zone=us-central1-a`. This also applies to `gcloud compute scp`.

### viam-server "No such file or directory"

The AppImage-extracted binary needs the explicit linker. Use:
```bash
/lib64/ld-linux-x86-64.so.2 /opt/squashfs-root/usr/bin/viam-server -config /opt/viam_config.json
```
Not just `/opt/viam-server-bin` (the symlink doesn't resolve the interpreter).

---

## Commit History

| Commit | Description |
|--------|-------------|
| `0e7949f` | Initial commit: xArm 6 Gazebo simulation with TCP protocol emulator |
| `766c0d7` | Add wrist-mounted Intel RealSense D435 RGBD camera |
| `1a66ae3` | Fix D435 camera mount position and restore 2ms physics timestep |
| `643f319` | Add viam-server integration and upgrade camera module to SDK 0.71.1 |
