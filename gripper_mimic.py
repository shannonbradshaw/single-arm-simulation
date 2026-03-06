"""Gazebo Python system plugin: mirrors drive_joint to all gripper linkage joints.

The xArm gripper is a parallel-jaw 4-bar linkage. The drive_joint actuates
the left outer knuckle; five other joints mimic it with various multipliers
to produce symmetric closure with parallel fingertips.

Bypasses the DART controller layer (which cannot actuate secondary gripper
joints) by setting joint positions directly each physics step.

Usage in SDF (model-level plugin):
  <plugin filename="gz-sim-python-system-loader-system"
          name="gz::sim::systems::PythonSystemLoader">
    <module_name>gripper_mimic</module_name>
  </plugin>
"""

import gz.sim8 as sim

# Joint name -> multiplier relative to drive_joint
# From xarm_description URDF mimic tags
MIMIC_JOINTS = {
    "left_inner_knuckle_joint":   1.0,
    "left_finger_joint":         -1.0,
    "right_outer_knuckle_joint": -1.0,
    "right_inner_knuckle_joint": -1.0,
    "right_finger_joint":         1.0,
}


class GripperMimic:
    def __init__(self):
        self.drive_entity = None
        self.mirrors = []  # list of (entity, multiplier)
        self.configured = False

    def configure(self, entity, sdf, ecm, event_mgr):
        model = sim.Model(entity)
        if not model.valid(ecm):
            print("[gripper_mimic] Model not valid", flush=True)
            return

        self.drive_entity = model.joint_by_name(ecm, "drive_joint")
        if self.drive_entity == sim.K_NULL_ENTITY:
            print("[gripper_mimic] drive_joint not found", flush=True)
            return

        for name, mult in MIMIC_JOINTS.items():
            ent = model.joint_by_name(ecm, name)
            if ent != sim.K_NULL_ENTITY:
                self.mirrors.append((ent, mult))
                print(f"[gripper_mimic] {name} -> x{mult}", flush=True)
            else:
                print(f"[gripper_mimic] {name} not found (skipped)", flush=True)

        self.configured = len(self.mirrors) > 0
        print(f"[gripper_mimic] Configured {len(self.mirrors)} mimic joints", flush=True)

    def pre_update(self, info, ecm):
        if not self.configured or info.paused:
            return

        drive = sim.Joint(self.drive_entity)
        if not drive.valid(ecm):
            return

        positions = drive.position(ecm)
        if not positions or len(positions) == 0:
            return

        theta = positions[0]
        for ent, mult in self.mirrors:
            joint = sim.Joint(ent)
            if joint.valid(ecm):
                joint.reset_position(ecm, [theta * mult])


def get_system():
    return GripperMimic()
