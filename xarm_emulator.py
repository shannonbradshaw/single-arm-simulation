"""xArm Protocol Emulator — bridges xArm TCP protocol to Gazebo gz-transport.

Emulates the UFactory xArm's Private Modbus TCP protocol, enabling the real
Viam UFactory xArm module to control the Gazebo simulation with zero code
changes. The only difference between sim and real is the target IP address.

Wire format (both request and response):
  [TransactionID:2 BE][ProtocolID:2 BE=0x0002][Length:2 BE][Register:1][Payload]
  Float payloads are IEEE 754 float32 little-endian.
  Header integers are big-endian.

Usage:
  python3 xarm_emulator.py

Environment variables:
  XARM_EMU_PORT          — TCP listen port (default: 502)
  XARM_EMU_TOPIC_PREFIX  — gz-transport topic prefix (default: /xarm6)
  XARM_EMU_VERBOSE       — set to "1" for protocol-level logging
"""

import os
import socket
import struct
import threading
import time

from gz.msgs10.model_pb2 import Model as GzModel
from gz.msgs10.double_pb2 import Double as GzDouble
from gz.transport13 import Node


# ─── Configuration ─────────────────────────────────────────────────────

PORT = int(os.environ.get("XARM_EMU_PORT", "502"))
DOF = 6
WIRE_JOINTS = 7  # xArm protocol always sends/receives 7 joint slots
VERBOSE = os.environ.get("XARM_EMU_VERBOSE", "0") == "1"
TOPIC_PREFIX = os.environ.get("XARM_EMU_TOPIC_PREFIX", "/xarm6")

# Threshold (radians) for determining if joints have reached their target
POSITION_THRESHOLD = 0.10

# Safe home position (radians) — gripper above table at startup
# [0, -45, -30, 0, 60, 0] degrees
import math
HOME_POSITIONS = [0.0, math.radians(-45), math.radians(-30),
                  0.0, math.radians(60), 0.0, 0.0]


# ─── Protocol Constants ───────────────────────────────────────────────

PROTO_PRIVATE = 0x0002

# Register IDs used by the Viam UFactory module
REG_GET_VERSION     = 0x01
REG_SYSTEM_CONTROL  = 0x0A
REG_MOTION_EN       = 0x0B
REG_SET_STATE       = 0x0C
REG_GET_STATE       = 0x0D
REG_GET_CMDNUM      = 0x0E
REG_GET_ERROR       = 0x0F
REG_CLEAN_ERR       = 0x10
REG_CLEAN_WAR       = 0x11
REG_SET_MODE        = 0x13
REG_MOVE_JOINT      = 0x17
REG_MOVE_HOME       = 0x19
REG_MOVE_SERVOJ     = 0x1D
REG_SET_COLLIS_SENS = 0x25
REG_GET_JOINT_POS   = 0x2A
REG_GET_JOINT_TAU   = 0x37
REG_SERVO_DBMSG     = 0x6A
REG_RS485_RTU       = 0x7C
REG_TGPIO_W16B      = 0x7F
REG_TGPIO_R16B      = 0x80

REG_NAMES = {v: k for k, v in dict(
    GET_VERSION=0x01, SYSTEM_CONTROL=0x0A, MOTION_EN=0x0B, SET_STATE=0x0C,
    GET_STATE=0x0D, GET_CMDNUM=0x0E, GET_ERROR=0x0F, CLEAN_ERR=0x10,
    CLEAN_WAR=0x11, SET_MODE=0x13, MOVE_JOINT=0x17, MOVE_HOME=0x19,
    MOVE_SERVOJ=0x1D, SET_COLLIS_SENS=0x25, GET_JOINT_POS=0x2A,
    GET_JOINT_TAU=0x37, SERVO_DBMSG=0x6A, RS485_RTU=0x7C,
    TGPIO_W16B=0x7F, TGPIO_R16B=0x80,
).items()}

# Gripper Modbus register addresses (via RS485 passthrough)
GRIP_CON_EN    = 0x0100
GRIP_TAGET_POS = 0x0700
GRIP_CURR_POS  = 0x0702
GRIP_POS_SPD   = 0x0303


def log(msg):
    print(f"[xarm-emu] {msg}", flush=True)


def vlog(msg):
    if VERBOSE:
        print(f"[xarm-emu] {msg}", flush=True)


# ─── Gazebo Bridge ─────────────────────────────────────────────────────

class GazeboBridge:
    """Manages gz-transport subscriptions and publishers for Gazebo."""

    def __init__(self):
        self.node = Node()
        self._joint_positions = [0.0] * WIRE_JOINTS
        self._joint_lock = threading.Lock()
        self._gripper_pos = 0  # 0-850 protocol range
        self._gripper_lock = threading.Lock()
        self._publishers = {}

        # Subscribe to joint states
        topic = f"{TOPIC_PREFIX}/joint_states"
        ok = self.node.subscribe(GzModel, topic, self._on_joint_states)
        log(f"Subscribe {topic}: {'OK' if ok else 'FAILED'}")

        # Create publishers for each joint + gripper
        for i in range(1, DOF + 1):
            t = f"{TOPIC_PREFIX}/joint{i}/cmd_pos"
            pub = self.node.advertise(t, GzDouble)
            self._publishers[t] = pub
            log(f"Advertise {t}: OK")

        grip_topic = f"{TOPIC_PREFIX}/gripper/cmd_pos"
        self._publishers[grip_topic] = self.node.advertise(grip_topic, GzDouble)
        log(f"Advertise {grip_topic}: OK")

    def _on_joint_states(self, msg):
        """Callback for Gazebo joint state messages."""
        try:
            with self._joint_lock:
                for joint in msg.joint:
                    name = joint.name
                    if name.startswith("joint") and name[5:].isdigit():
                        idx = int(name[5:]) - 1
                        if 0 <= idx < DOF:
                            self._joint_positions[idx] = joint.axis1.position
                    elif name == "drive_joint":
                        with self._gripper_lock:
                            # drive_joint range ~0-0.85 rad → protocol 0-850
                            self._gripper_pos = max(0, min(850,
                                int(joint.axis1.position * 1000)))
        except Exception as e:
            log(f"Joint state error: {e}")

    def get_joint_positions(self):
        """Return current joint positions (7 floats, radians)."""
        with self._joint_lock:
            return list(self._joint_positions)

    def get_gripper_position(self):
        """Return gripper position in protocol range (0-850)."""
        with self._gripper_lock:
            return self._gripper_pos

    def set_joint_positions(self, positions):
        """Publish joint position commands to Gazebo (radians)."""
        for i in range(min(DOF, len(positions))):
            topic = f"{TOPIC_PREFIX}/joint{i+1}/cmd_pos"
            pub = self._publishers.get(topic)
            if pub:
                msg = GzDouble()
                msg.data = positions[i]
                pub.publish(msg)

    def set_gripper_position(self, value):
        """Publish gripper command (protocol 0-850 → joint 0-0.85 rad)."""
        topic = f"{TOPIC_PREFIX}/gripper/cmd_pos"
        pub = self._publishers.get(topic)
        if pub:
            msg = GzDouble()
            msg.data = max(0.0, min(0.85, value / 1000.0))
            pub.publish(msg)


# ─── Arm State ─────────────────────────────────────────────────────────

class ArmState:
    """Tracks simulated arm state (mode, servos, targets)."""

    def __init__(self):
        self.mode = 0               # 0=position, 1=servo
        self.state = 0              # 0=ready, 3=suspend, 4=stop
        self.servos_enabled = False
        self.target_positions = list(HOME_POSITIONS)
        self.gripper_enabled = False
        self.gripper_target = 0
        self.last_move_time = 0.0   # timestamp of last move command
        self.lock = threading.Lock()

    # Seconds to wait after last move command before reporting "idle"
    SETTLE_TIME = 3.0

    def is_moving(self, current_positions):
        """Check if arm is still settling after a move command.

        Returns True (moving) for SETTLE_TIME seconds after the last move
        command, then False (idle). This lets the PID controllers settle
        before telling the driver the move is complete.
        """
        with self.lock:
            if self.last_move_time == 0.0:
                return False
            elapsed = time.time() - self.last_move_time
            return elapsed < self.SETTLE_TIME


# ─── Protocol Handler ──────────────────────────────────────────────────

class ProtocolHandler:
    """Handles the xArm Private Modbus TCP protocol for one client."""

    def __init__(self, conn, bridge, arm_state):
        self.conn = conn
        self.gz = bridge
        self.arm = arm_state

    def run(self):
        """Read commands, dispatch, respond — until disconnect."""
        try:
            while True:
                # Read 7-byte header
                header = self._recv(7)
                if header is None:
                    break

                tid = struct.unpack_from(">H", header, 0)[0]
                length = struct.unpack_from(">H", header, 4)[0]
                reg = header[6]

                # Read remaining payload (length includes the register byte)
                params = b""
                if length > 1:
                    params = self._recv(length - 1)
                    if params is None:
                        break

                vlog(f"<- {REG_NAMES.get(reg, f'0x{reg:02X}')} "
                     f"tid={tid} params={len(params)}B")

                response = self._dispatch(tid, reg, params)
                self.conn.sendall(response)

        except (ConnectionResetError, BrokenPipeError):
            pass
        except Exception as e:
            log(f"Protocol error: {e}")

    def _recv(self, n):
        """Read exactly n bytes from socket."""
        data = b""
        while len(data) < n:
            chunk = self.conn.recv(n - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _resp(self, tid, reg, payload=b"\x00"):
        """Build response frame. First byte of payload is the state byte."""
        length = 1 + len(payload)
        return struct.pack(">HHH", tid, PROTO_PRIVATE, length) + \
            bytes([reg]) + payload

    # ── Command dispatch ──────────────────────────────────────────────

    def _dispatch(self, tid, reg, params):
        handlers = {
            REG_GET_VERSION:     self._get_version,
            REG_SYSTEM_CONTROL:  self._ack,
            REG_MOTION_EN:       self._motion_en,
            REG_SET_STATE:       self._set_state,
            REG_GET_STATE:       self._get_state,
            REG_GET_CMDNUM:      self._get_cmdnum,
            REG_GET_ERROR:       self._get_error,
            REG_CLEAN_ERR:       self._ack,
            REG_CLEAN_WAR:       self._ack,
            REG_SET_MODE:        self._set_mode,
            REG_MOVE_JOINT:      self._move_joints,
            REG_MOVE_HOME:       self._move_home,
            REG_MOVE_SERVOJ:     self._move_joints,
            REG_SET_COLLIS_SENS: self._ack,
            REG_GET_JOINT_POS:   self._get_joint_pos,
            REG_GET_JOINT_TAU:   self._get_joint_tau,
            REG_SERVO_DBMSG:     self._servo_dbmsg,
            REG_RS485_RTU:       self._gripper,
            REG_TGPIO_W16B:      self._ack,
            REG_TGPIO_R16B:      self._tgpio_read,
        }
        handler = handlers.get(reg)
        if handler:
            return handler(tid, reg, params)
        vlog(f"Unknown register 0x{reg:02X} — returning OK")
        return self._resp(tid, reg)

    # ── Handlers ──────────────────────────────────────────────────────

    def _ack(self, tid, reg, params):
        """Simple acknowledgment — state byte 0x00 (no errors)."""
        return self._resp(tid, reg)

    def _get_version(self, tid, reg, params):
        version = b"xArm6 Gazebo Emulator v1.0\x00"
        version = version[:40].ljust(40, b"\x00")
        return self._resp(tid, reg, b"\x00" + version)

    def _motion_en(self, tid, reg, params):
        if len(params) >= 2:
            with self.arm.lock:
                self.arm.servos_enabled = (params[1] == 1)
            vlog(f"Servos {'ON' if params[1] == 1 else 'OFF'}")
        return self._resp(tid, reg)

    def _set_state(self, tid, reg, params):
        if len(params) >= 1:
            with self.arm.lock:
                self.arm.state = params[0]
            vlog(f"State -> {params[0]}")
        return self._resp(tid, reg)

    def _get_state(self, tid, reg, params):
        current = self.gz.get_joint_positions()
        moving = self.arm.is_moving(current)
        # params[0]=state_byte, params[1]=motion_state (0x01=moving, 0x02=idle)
        motion_byte = 0x01 if moving else 0x02
        return self._resp(tid, reg, bytes([0x00, motion_byte]))

    def _get_cmdnum(self, tid, reg, params):
        # Always 0 pending commands
        return self._resp(tid, reg, b"\x00" + struct.pack(">H", 0))

    def _get_error(self, tid, reg, params):
        # state=0x00 (no flags), error_code=0, warning_code=0
        return self._resp(tid, reg, b"\x00\x00\x00")

    def _set_mode(self, tid, reg, params):
        if len(params) >= 1:
            with self.arm.lock:
                self.arm.mode = params[0]
            vlog(f"Mode -> {params[0]}")
        return self._resp(tid, reg)

    def _move_joints(self, tid, reg, params):
        """Handle MOVE_SERVOJ (0x1D) and MOVE_JOINT (0x17).

        Payload: 7 x float32 LE (joint radians) + speed + accel + time = 40 bytes.
        """
        if len(params) >= WIRE_JOINTS * 4:
            joints = []
            for i in range(WIRE_JOINTS):
                val = struct.unpack_from("<f", params, i * 4)[0]
                joints.append(val)

            with self.arm.lock:
                self.arm.target_positions = list(joints)
                self.arm.last_move_time = time.time()

            self.gz.set_joint_positions(joints[:DOF])
            vlog(f"Joints -> [{', '.join(f'{j:.3f}' for j in joints[:DOF])}]")
        return self._resp(tid, reg)

    def _move_home(self, tid, reg, params):
        zeros = [0.0] * WIRE_JOINTS
        with self.arm.lock:
            self.arm.target_positions = zeros
        self.gz.set_joint_positions([0.0] * DOF)
        vlog("Home -> all zeros")
        return self._resp(tid, reg)

    def _get_joint_pos(self, tid, reg, params):
        """Return 7 joint positions as float32 LE.

        Returns the last commanded target positions rather than actual Gazebo
        positions. A real xArm's internal servo holds commanded positions
        near-perfectly; in sim the PID controllers drift under gravity, which
        causes the ufactory module's interpolation to accumulate error.
        """
        with self.arm.lock:
            positions = list(self.arm.target_positions)
        data = b"\x00"  # state byte
        for pos in positions:
            data += struct.pack("<f", pos)
        return self._resp(tid, reg, data)

    def _get_joint_tau(self, tid, reg, params):
        """Return 7 zero torque values."""
        data = b"\x00" + (b"\x00\x00\x00\x00" * WIRE_JOINTS)
        return self._resp(tid, reg, data)

    def _servo_dbmsg(self, tid, reg, params):
        """No servo errors — 16 zero bytes."""
        return self._resp(tid, reg, b"\x00" + b"\x00" * 16)

    def _gripper(self, tid, reg, params):
        """Handle RS485_RTU gripper passthrough (Modbus RTU to gripper servo).

        Params: [host_id=0x09, device_id=0x08, modbus_func, ...modbus_pdu...]
        """
        if len(params) < 3:
            return self._resp(tid, reg)

        host_id, device_id, modbus_func = params[0], params[1], params[2]

        if modbus_func == 0x10 and len(params) >= 10:
            # Write multiple registers
            reg_addr = struct.unpack_from(">H", params, 3)[0]

            if reg_addr == GRIP_CON_EN:
                val = struct.unpack_from(">H", params, 8)[0]
                with self.arm.lock:
                    self.arm.gripper_enabled = (val == 1)
                vlog(f"Gripper {'enabled' if val == 1 else 'disabled'}")

            elif reg_addr == GRIP_TAGET_POS and len(params) >= 12:
                val = struct.unpack_from(">i", params, 8)[0]
                with self.arm.lock:
                    self.arm.gripper_target = val
                self.gz.set_gripper_position(val)
                vlog(f"Gripper -> {val}")

            elif reg_addr == GRIP_POS_SPD:
                vlog("Gripper speed set (ignored)")

            # Write confirmation echo
            echo = bytes([0x00, host_id, device_id, modbus_func])
            if len(params) >= 7:
                echo += params[3:7]
            return self._resp(tid, reg, echo)

        elif modbus_func == 0x03 and len(params) >= 7:
            # Read holding registers
            reg_addr = struct.unpack_from(">H", params, 3)[0]

            if reg_addr == GRIP_CURR_POS:
                pos = self.gz.get_gripper_position()
                data = bytes([0x00, host_id, device_id, modbus_func, 0x04])
                data += struct.pack(">i", pos)
                return self._resp(tid, reg, data)

            # Default read: 4 zero bytes
            data = bytes([0x00, host_id, device_id, modbus_func, 0x04,
                          0, 0, 0, 0])
            return self._resp(tid, reg, data)

        return self._resp(tid, reg)

    def _tgpio_read(self, tid, reg, params):
        """Tool GPIO read — return 5 zero bytes (state + 4 data)."""
        return self._resp(tid, reg, b"\x00\x00\x00\x00\x00")


# ─── TCP Server ────────────────────────────────────────────────────────

def serve():
    bridge = GazeboBridge()
    arm_state = ArmState()

    # Publish safe home position so arm starts above the table
    time.sleep(1)  # let gz-transport topics settle
    bridge.set_joint_positions(HOME_POSITIONS[:DOF])
    log(f"Published home position: {[f'{math.degrees(p):.0f}°' for p in HOME_POSITIONS[:DOF]]}")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", PORT))
    server.listen(2)
    log(f"Listening on 0.0.0.0:{PORT}")

    while True:
        conn, addr = server.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        log(f"Client connected: {addr}")

        def handle(c=conn, a=addr):
            handler = ProtocolHandler(c, bridge, arm_state)
            handler.run()
            c.close()
            log(f"Client disconnected: {a}")

        threading.Thread(target=handle, daemon=True).start()


if __name__ == "__main__":
    log("Starting xArm protocol emulator...")
    log(f"Topics: {TOPIC_PREFIX}/joint_states, {TOPIC_PREFIX}/joint{{1-6}}/cmd_pos")
    serve()
