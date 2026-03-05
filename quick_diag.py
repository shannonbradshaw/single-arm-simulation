"""Quick diagnostic: move to home, read Gazebo state."""
import asyncio, math, os, subprocess, sys
from viam.robot.client import RobotClient
from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions

HOME = [0.0, -45.0, -30.0, 0.0, 60.0, 0.0]

def read_gz():
    cmd = "gcloud compute ssh shannon.bradshaw@xarm6-sim --zone=us-central1-a --command=\"sudo docker exec xarm6-full gz topic --topic /xarm6/joint_states --echo --num 1\""
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=30)
        lines = r.stdout.split("\n")
        joint = None
        positions = {}
        for line in lines:
            line = line.strip()
            if 'name: "joint' in line or 'name: "drive_joint"' in line:
                joint = line.split('"')[1]
            elif joint and line.startswith("position:") and "}" not in line:
                try:
                    val = float(line.split(":")[1].strip())
                    positions[joint] = math.degrees(val)
                    joint = None
                except:
                    pass
        return positions
    except:
        return None

async def main():
    opts = RobotClient.Options.with_api_key(
        os.environ["VIAM_API_KEY"], os.environ["VIAM_API_KEY_ID"])
    machine = await RobotClient.at_address(os.environ["VIAM_ADDRESS"], opts)
    arm = Arm.from_robot(robot=machine, name="arm")

    # Read current SDK position
    pos = await arm.get_joint_positions()
    print(f"SDK before move: {[f'{v:.1f}' for v in pos.values]}")

    # Read Gazebo before move
    gz = read_gz()
    if gz:
        print(f"Gazebo before move:")
        for j in ["joint1","joint2","joint3","joint4","joint5","joint6"]:
            if j in gz:
                idx = int(j[-1]) - 1
                err = abs(gz[j] - HOME[idx])
                print(f"  {j}: {gz[j]:8.2f}° (home={HOME[idx]:.0f}°, err={err:.1f}°)")

    # Move to home
    print("\nMoving to home...")
    await arm.move_to_joint_positions(JointPositions(values=HOME), timeout=30)
    print("Move complete. Waiting 6s to settle...")
    await asyncio.sleep(6)

    # Read SDK after
    pos = await arm.get_joint_positions()
    print(f"\nSDK after move: {[f'{v:.1f}' for v in pos.values]}")

    # Read Gazebo after
    gz = read_gz()
    if gz:
        print(f"Gazebo after move:")
        for j in ["joint1","joint2","joint3","joint4","joint5","joint6"]:
            if j in gz:
                idx = int(j[-1]) - 1
                err = abs(gz[j] - HOME[idx])
                status = "✓" if err < 3 else "✗"
                print(f"  {j}: {gz[j]:8.2f}° (home={HOME[idx]:.0f}°, err={err:.1f}°) {status}")
    else:
        print("WARNING: Could not read Gazebo state")

    # Now try moving J5 specifically to different values
    print("\n--- Testing J5 specifically ---")
    for j5_target in [0, 30, 60, 90]:
        target = list(HOME)
        target[4] = j5_target
        print(f"\nMoving J5 to {j5_target}°...")
        await arm.move_to_joint_positions(JointPositions(values=target), timeout=30)
        await asyncio.sleep(4)
        gz = read_gz()
        if gz and "joint5" in gz:
            err = abs(gz["joint5"] - j5_target)
            status = "✓" if err < 3 else "✗"
            print(f"  J5 Gazebo: {gz['joint5']:.2f}° (target={j5_target}°, err={err:.1f}°) {status}")
        else:
            pos = await arm.get_joint_positions()
            print(f"  J5 SDK: {pos.values[4]:.1f}° (no Gazebo data)")

    await machine.close()

if __name__ == "__main__":
    asyncio.run(main())
