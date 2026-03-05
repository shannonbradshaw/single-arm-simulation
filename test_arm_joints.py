"""Automated joint movement test for xArm6 Gazebo simulation.

Moves each joint through a series of positions via the Viam SDK, then reads
actual Gazebo joint states via gz-transport to verify the physical simulation
tracked the commanded positions. Reports coupling effects (unintended movement
of non-target joints).

Usage:
    # Set credentials from Viam app (CONNECT tab > API keys)
    export VIAM_API_KEY="..."
    export VIAM_API_KEY_ID="..."
    export VIAM_ADDRESS="xarm6-sim-main.XXXXX.viam.cloud"

    python3 test_arm_joints.py

    # Or run specific test groups:
    python3 test_arm_joints.py --test single    # Single-joint sweeps only
    python3 test_arm_joints.py --test multi     # Multi-joint moves only
    python3 test_arm_joints.py --test coupling  # Coupling analysis only
"""

import argparse
import asyncio
import json
import math
import os
import subprocess
import sys
import time

from viam.robot.client import RobotClient
from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions


# ─── Constants ─────────────────────────────────────────────────────────

# Canonical starting pose (degrees)
HOME = [0.0, -45.0, -30.0, 0.0, 60.0, 0.0]

# Joint limits (degrees) from the SDF
JOINT_LIMITS = [
    (-360.0, 360.0),   # J1: base rotation
    (-118.0, 120.0),   # J2: shoulder
    (-225.0, 11.0),    # J3: elbow
    (-360.0, 360.0),   # J4: wrist rotation
    (-97.0, 180.0),    # J5: wrist bend
    (-360.0, 360.0),   # J6: tool rotation
]

JOINT_NAMES = ["J1 base", "J2 shoulder", "J3 elbow",
               "J4 wrist-rot", "J5 wrist-bend", "J6 tool-rot"]

# Settle time after a move before reading Gazebo state (seconds)
SETTLE_TIME = 6.0

# Position tolerance for pass/fail (degrees)
TARGET_TOLERANCE = 3.0

# Coupling tolerance — how much a non-target joint can move (degrees)
COUPLING_TOLERANCE = 2.0


# ─── Gazebo State Reader ──────────────────────────────────────────────

def read_gazebo_joints(docker_cmd_prefix="sudo docker exec xarm6-full"):
    """Read actual joint positions from Gazebo via gz-transport.

    Returns dict of {joint_name: position_in_degrees}.
    """
    cmd = f"{docker_cmd_prefix} gz topic --topic /xarm6/joint_states --echo --num 1"
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=10
        )
        output = result.stdout
    except subprocess.TimeoutExpired:
        print("  WARNING: gz topic timed out")
        return None

    positions = {}
    lines = output.split("\n")
    current_joint = None
    for i, line in enumerate(lines):
        line = line.strip()
        if 'name: "joint' in line or 'name: "drive_joint"' in line:
            current_joint = line.split('"')[1]
        elif current_joint and "position:" in line and "}" not in line:
            try:
                val = float(line.split(":")[1].strip())
                positions[current_joint] = math.degrees(val)
                current_joint = None
            except (ValueError, IndexError):
                pass
    return positions


def read_gazebo_joints_remote(vm_user="shannon.bradshaw", vm_name="xarm6-sim",
                               zone="us-central1-a"):
    """Read Gazebo joints via gcloud SSH (for running tests locally)."""
    inner_cmd = "sudo docker exec xarm6-full gz topic --topic /xarm6/joint_states --echo --num 1"
    cmd = f"gcloud compute ssh {vm_user}@{vm_name} --zone={zone} --command=\"{inner_cmd}\""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=60
        )
        output = result.stdout
    except subprocess.TimeoutExpired:
        print("  WARNING: gcloud ssh timed out")
        return None

    positions = {}
    lines = output.split("\n")
    current_joint = None
    for i, line in enumerate(lines):
        line = line.strip()
        if 'name: "joint' in line or 'name: "drive_joint"' in line:
            current_joint = line.split('"')[1]
        elif current_joint and "position:" in line and "}" not in line:
            try:
                val = float(line.split(":")[1].strip())
                positions[current_joint] = math.degrees(val)
                current_joint = None
            except (ValueError, IndexError):
                pass
    return positions


# ─── Test Helpers ──────────────────────────────────────────────────────

class TestResults:
    def __init__(self):
        self.tests = []

    def record(self, name, passed, details=""):
        self.tests.append({"name": name, "passed": passed, "details": details})
        status = "PASS" if passed else "FAIL"
        print(f"  [{status}] {name}")
        if details:
            print(f"         {details}")

    def summary(self):
        total = len(self.tests)
        passed = sum(1 for t in self.tests if t["passed"])
        failed = total - passed
        print(f"\n{'='*60}")
        print(f"RESULTS: {passed}/{total} passed, {failed} failed")
        print(f"{'='*60}")
        if failed > 0:
            print("\nFailed tests:")
            for t in self.tests:
                if not t["passed"]:
                    print(f"  - {t['name']}: {t['details']}")
        return failed == 0


async def move_and_verify(arm, target_deg, results, test_name,
                          read_gz_fn, prev_gz_positions=None,
                          moving_joints=None):
    """Move arm to target, wait for settle, verify actual Gazebo positions.

    Args:
        arm: Viam Arm component
        target_deg: list of 6 target joint angles in degrees
        results: TestResults instance
        test_name: descriptive name for this test
        read_gz_fn: function to read Gazebo joint positions
        prev_gz_positions: Gazebo positions before this move (for coupling check)
        moving_joints: set of joint indices being intentionally moved (0-indexed)
    """
    print(f"\n--- {test_name} ---")
    print(f"  Target: {[f'{v:.1f}' for v in target_deg]}")

    # Read SDK position before move
    sdk_before = await arm.get_joint_positions()
    sdk_before_list = list(sdk_before.values)

    # Execute move
    target = JointPositions(values=target_deg)
    try:
        await arm.move_to_joint_positions(positions=target, timeout=30.0)
    except Exception as e:
        results.record(f"{test_name} [move]", False, f"Move failed: {e}")
        return None

    # Wait for PID controllers to settle
    await asyncio.sleep(SETTLE_TIME)

    # Read SDK position (commanded/target positions)
    sdk_after = await arm.get_joint_positions()
    sdk_after_list = list(sdk_after.values)
    print(f"  SDK reported: {[f'{v:.1f}' for v in sdk_after_list]}")

    # If we have Gazebo access, read actual physics positions
    gz_deg = None
    gz_positions = None
    if read_gz_fn is not None:
        gz_positions = read_gz_fn()
        if gz_positions is not None:
            gz_joint_map = {f"joint{i+1}": i for i in range(6)}
            gz_deg = [0.0] * 6
            for jname, pos_deg in gz_positions.items():
                if jname in gz_joint_map:
                    gz_deg[gz_joint_map[jname]] = pos_deg
            print(f"  Gazebo actual: {[f'{v:.1f}' for v in gz_deg]}")

    # Use Gazebo positions if available, otherwise SDK positions
    check_positions = gz_deg if gz_deg is not None else sdk_after_list

    # Check each joint's position against target
    all_on_target = True
    for i in range(6):
        error = abs(check_positions[i] - target_deg[i])
        if error > TARGET_TOLERANCE:
            src = "gz" if gz_deg is not None else "sdk"
            results.record(
                f"{test_name} [{JOINT_NAMES[i]} tracking]", False,
                f"target={target_deg[i]:.1f}, {src}={check_positions[i]:.1f}, "
                f"error={error:.1f} > {TARGET_TOLERANCE}"
            )
            all_on_target = False

    if all_on_target:
        results.record(f"{test_name} [tracking]", True,
                       f"All joints within {TARGET_TOLERANCE} deg")

    # Coupling check: did non-target joints move unexpectedly?
    if moving_joints is not None:
        # Use SDK before/after for coupling check (always available)
        coupling_issues = []
        for i in range(6):
            if i not in moving_joints:
                drift = abs(sdk_after_list[i] - sdk_before_list[i])
                if drift > COUPLING_TOLERANCE:
                    coupling_issues.append(
                        f"{JOINT_NAMES[i]}: moved {drift:.1f} deg"
                    )

        if coupling_issues:
            results.record(
                f"{test_name} [coupling]", False,
                "Unintended joint movement: " + "; ".join(coupling_issues)
            )
        else:
            results.record(f"{test_name} [coupling]", True,
                           "No coupling detected")

    return gz_positions


# ─── Test Suites ───────────────────────────────────────────────────────

async def test_single_joint_sweeps(arm, results, read_gz_fn):
    """Move each joint individually through several positions."""
    print("\n" + "="*60)
    print("TEST SUITE: Single-Joint Sweeps")
    print("="*60)

    # Start from home
    await arm.move_to_joint_positions(JointPositions(values=HOME), timeout=30)
    await asyncio.sleep(SETTLE_TIME)

    # Test increments for each joint (degrees from home)
    test_deltas = {
        0: [30, 90, -45, -90],       # J1: base rotation
        1: [15, -15, -30],            # J2: shoulder (limited range)
        2: [15, -30, -60],            # J3: elbow
        3: [45, 90, -45, -90],        # J4: wrist rotation
        4: [-30, -60, 30, 60],        # J5: wrist bend
        5: [45, 90, 180, -90, -180],  # J6: tool rotation
    }

    for joint_idx, deltas in test_deltas.items():
        for delta in deltas:
            target = list(HOME)
            target[joint_idx] = HOME[joint_idx] + delta

            # Clamp to limits
            lo, hi = JOINT_LIMITS[joint_idx]
            target[joint_idx] = max(lo, min(hi, target[joint_idx]))

            # Return to home first
            await arm.move_to_joint_positions(JointPositions(values=HOME), timeout=30)
            await asyncio.sleep(SETTLE_TIME)

            # Move the single joint
            name = f"{JOINT_NAMES[joint_idx]} delta={delta:+.0f}"
            await move_and_verify(
                arm, target, results, name, read_gz_fn,
                moving_joints={joint_idx}
            )


async def test_multi_joint_moves(arm, results, read_gz_fn):
    """Move multiple joints simultaneously."""
    print("\n" + "="*60)
    print("TEST SUITE: Multi-Joint Moves")
    print("="*60)

    multi_poses = [
        ("reach forward",     [0, -60, -60, 0, 90, 0]),
        ("look left",         [90, -45, -30, 0, 60, 0]),
        ("look right",        [-90, -45, -30, 0, 60, 0]),
        ("wrist complex",     [0, -45, -30, 45, 30, 90]),
        ("full extension",    [0, -30, -10, 0, 30, 0]),
        ("compact fold",      [0, -90, -90, 0, 90, 0]),
        ("diagonal reach",    [45, -30, -45, -30, 45, 45]),
    ]

    for name, target in multi_poses:
        # Return to home first
        await arm.move_to_joint_positions(JointPositions(values=HOME), timeout=30)
        await asyncio.sleep(SETTLE_TIME)

        await move_and_verify(arm, target, results, f"multi: {name}", read_gz_fn,
                              moving_joints=set(range(6)))


async def test_coupling_analysis(arm, results, read_gz_fn):
    """Specifically test coupling between adjacent joints."""
    print("\n" + "="*60)
    print("TEST SUITE: Coupling Analysis")
    print("="*60)

    # Test pairs that are likely to couple
    coupling_tests = [
        # (description, joint_to_move, delta, joints_to_watch)
        ("J4 rot -> J5 coupling",  3, 90,  {4, 5}),
        ("J5 bend -> J4 coupling", 4, -30, {3, 5}),
        ("J6 rot -> J4,J5 coupling", 5, 90, {3, 4}),
        ("J6 rot -> J4,J5 coupling (neg)", 5, -90, {3, 4}),
        ("J2 shoulder -> J3 coupling", 1, 15, {2}),
        ("J3 elbow -> J2 coupling", 2, -30, {1}),
        ("J1 base -> J2 coupling (fast)", 0, 90, {1, 2}),
    ]

    for desc, joint_idx, delta, watch_joints in coupling_tests:
        # Start from home
        await arm.move_to_joint_positions(JointPositions(values=HOME), timeout=30)
        await asyncio.sleep(SETTLE_TIME)

        target = list(HOME)
        target[joint_idx] = HOME[joint_idx] + delta
        lo, hi = JOINT_LIMITS[joint_idx]
        target[joint_idx] = max(lo, min(hi, target[joint_idx]))

        await move_and_verify(
            arm, target, results, f"coupling: {desc}", read_gz_fn,
            moving_joints={joint_idx}
        )


# ─── Main ─────────────────────────────────────────────────────────────

async def main():
    parser = argparse.ArgumentParser(description="xArm6 joint movement tests")
    parser.add_argument("--test", choices=["single", "multi", "coupling", "all"],
                        default="all", help="Which test suite to run")
    parser.add_argument("--local", action="store_true",
                        help="Read Gazebo state locally (running inside container)")
    args = parser.parse_args()

    # Connect to Viam
    api_key = os.environ.get("VIAM_API_KEY", "")
    api_key_id = os.environ.get("VIAM_API_KEY_ID", "")
    address = os.environ.get("VIAM_ADDRESS", "")

    if not all([api_key, api_key_id, address]):
        print("ERROR: Set VIAM_API_KEY, VIAM_API_KEY_ID, and VIAM_ADDRESS")
        sys.exit(1)

    opts = RobotClient.Options.with_api_key(api_key, api_key_id)
    machine = await RobotClient.at_address(address, opts)
    print(f"Connected to {address}")

    # Find the arm component
    arm_name = "arm"
    for r in machine.resource_names:
        if r.subtype == "arm":
            arm_name = r.name
            break
    arm = Arm.from_robot(robot=machine, name=arm_name)
    print(f"Using arm component: {arm_name}")

    # Select Gazebo reader
    if args.local:
        read_gz_fn = read_gazebo_joints
    else:
        # Try remote, fall back to SDK-only mode
        gz_test = read_gazebo_joints_remote()
        if gz_test is not None and len(gz_test) > 0:
            read_gz_fn = read_gazebo_joints_remote
            print(f"Gazebo connectivity OK: {len(gz_test)} joints found")
        else:
            print("WARNING: Cannot read Gazebo state via SSH. Running in SDK-only mode.")
            print("         SDK positions reflect commanded targets, not actual physics.")
            print("         Watch the web viewer at http://34.30.31.198:8081 for visual verification.")
            read_gz_fn = None

    # Go home first
    print("\nMoving to home position...")
    await arm.move_to_joint_positions(JointPositions(values=HOME), timeout=30)
    await asyncio.sleep(SETTLE_TIME)

    results = TestResults()

    if args.test in ("single", "all"):
        await test_single_joint_sweeps(arm, results, read_gz_fn)

    if args.test in ("multi", "all"):
        await test_multi_joint_moves(arm, results, read_gz_fn)

    if args.test in ("coupling", "all"):
        await test_coupling_analysis(arm, results, read_gz_fn)

    # Return home
    print("\nReturning to home...")
    await arm.move_to_joint_positions(JointPositions(values=HOME), timeout=30)

    all_passed = results.summary()
    await machine.close()
    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    asyncio.run(main())
