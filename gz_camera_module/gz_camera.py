"""Simulated Intel RealSense D435 — Viam Camera component backed by gz-transport.

Drop-in replacement for viam:camera:realsense. Subscribes to separate Gazebo
color camera (69° HFOV) and depth camera (87° HFOV) sensors, matching the real
D435's dual-sensor architecture. Serves images through the same Camera API as
the real RealSense module.

Accepts the same config attributes (sensors, width_px, height_px) so the Viam
app config works for both sim and real hardware.

Sim-only attribute:
    topic_prefix (str): gz-transport topic prefix, default "/d435"
"""

import math
import io
import struct
import threading
from typing import Any, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
from PIL import Image as PILImage

from viam.components.camera import Camera, DistortionParameters, IntrinsicParameters
from viam.media.video import CameraMimeType, NamedImage, ViamImage
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResponseMetadata
from viam.resource.base import ResourceBase
from viam.resource.types import Model, ModelFamily

from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node

# D435 horizontal FOV (radians)
COLOR_HFOV = 1.2043   # 69 degrees
DEPTH_HFOV = 1.5184   # 87 degrees
MAX_GRPC_MESSAGE_SIZE = 33554432  # 32 MB


class GzCamera(Camera, Reconfigurable):
    MODEL: ClassVar[Model] = Model(
        ModelFamily("viam", "camera"), "realsense"
    )

    def __init__(self, name: str):
        super().__init__(name)
        self._node: Optional[Node] = None
        self._topic_prefix = "/d435"
        self._width = 1280
        self._height = 720
        self._sensors: List[str] = ["color", "depth"]

        self._color_lock = threading.Lock()
        self._color_data: Optional[bytes] = None
        self._color_width = 0
        self._color_height = 0

        self._depth_lock = threading.Lock()
        self._depth_data: Optional[bytes] = None
        self._depth_width = 0
        self._depth_height = 0

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceBase, Any]
    ) -> "GzCamera":
        camera = cls(config.name)
        camera.reconfigure(config, dependencies)
        return camera

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceBase, Any]
    ):
        attrs = dict(config.attributes.fields)

        # Parse sensors list (matches real realsense module config)
        sensors_val = attrs.get("sensors")
        if sensors_val and sensors_val.list_value and sensors_val.list_value.values:
            self._sensors = [v.string_value for v in sensors_val.list_value.values]
        else:
            self._sensors = ["color", "depth"]

        w = attrs.get("width_px")
        self._width = int(w.number_value) if w and w.number_value else 1280
        h = attrs.get("height_px")
        self._height = int(h.number_value) if h and h.number_value else 720

        # Sim-only: gz-transport topic prefix
        tp = attrs.get("topic_prefix")
        self._topic_prefix = tp.string_value if tp else "/d435"

        self._setup_subscriptions()

    def _setup_subscriptions(self):
        self._node = Node()

        if "color" in self._sensors:
            topic = f"{self._topic_prefix}/color/image"
            ok = self._node.subscribe(GzImage, topic, self._on_color)
            print(f"[gz-camera] Subscribe {topic}: {'OK' if ok else 'FAILED'}",
                  flush=True)

        if "depth" in self._sensors:
            topic = f"{self._topic_prefix}/depth/image"
            ok = self._node.subscribe(GzImage, topic, self._on_depth)
            print(f"[gz-camera] Subscribe {topic}: {'OK' if ok else 'FAILED'}",
                  flush=True)

    def _on_color(self, msg: GzImage):
        with self._color_lock:
            self._color_data = msg.data
            self._color_width = msg.width
            self._color_height = msg.height

    def _on_depth(self, msg: GzImage):
        with self._depth_lock:
            self._depth_data = msg.data
            self._depth_width = msg.width
            self._depth_height = msg.height

    def _get_color_jpeg_bytes(self) -> bytes:
        with self._color_lock:
            if self._color_data is None:
                raise RuntimeError("No color frame received yet")
            data = self._color_data
            w = self._color_width
            h = self._color_height

        img = PILImage.frombytes("RGB", (w, h), data)
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=75)
        return buf.getvalue()

    def _depth_to_viam_format(self) -> bytes:
        """Convert Gazebo R_FLOAT32 depth to Viam depth map.

        Viam depth format: "VIAM_DPTH" magic (9 bytes) + width (8 bytes LE) +
        height (8 bytes LE) + pixel data (width*height uint16 big-endian, mm).
        """
        with self._depth_lock:
            if self._depth_data is None:
                raise RuntimeError("No depth frame received yet")
            raw = self._depth_data
            w = self._depth_width
            h = self._depth_height

        num_pixels = w * h
        floats = struct.unpack(f"<{num_pixels}f", raw)

        header = b"VIAM_DPTH"
        header += struct.pack("<QQ", w, h)

        pixels = bytearray(num_pixels * 2)
        for i, depth_m in enumerate(floats):
            if depth_m <= 0 or depth_m != depth_m or math.isinf(depth_m):
                mm = 0
            else:
                mm = min(65535, int(depth_m * 1000))
            struct.pack_into(">H", pixels, i * 2, mm)

        return header + bytes(pixels)

    # -- Camera API methods matching viam:camera:realsense --

    async def get_image(
        self,
        mime_type: str = "",
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> ViamImage:
        main = self._sensors[0] if self._sensors else "color"
        if main == "color":
            return ViamImage(self._get_color_jpeg_bytes(), CameraMimeType.JPEG)
        else:
            return ViamImage(self._depth_to_viam_format(), CameraMimeType.VIAM_RAW_DEPTH)

    async def get_images(
        self,
        *,
        filter_source_names: Optional[Sequence[str]] = None,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[List[NamedImage], ResponseMetadata]:
        wanted = set(filter_source_names) if filter_source_names else set(self._sensors)
        images: List[NamedImage] = []

        for sensor in self._sensors:
            if sensor not in wanted:
                continue
            if sensor == "color":
                try:
                    images.append(NamedImage(
                        "color", self._get_color_jpeg_bytes(), CameraMimeType.JPEG))
                except RuntimeError:
                    pass
            elif sensor == "depth":
                try:
                    images.append(NamedImage(
                        "depth", self._depth_to_viam_format(), CameraMimeType.VIAM_RAW_DEPTH))
                except RuntimeError:
                    pass

        return images, ResponseMetadata()

    async def get_point_cloud(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[bytes, str]:
        if "color" not in self._sensors or "depth" not in self._sensors:
            raise RuntimeError(
                "GetPointCloud requires both 'color' and 'depth' in sensors config")

        with self._color_lock:
            if self._color_data is None:
                raise RuntimeError("No color frame received yet")
            color_raw = self._color_data
            cw = self._color_width
            ch = self._color_height

        with self._depth_lock:
            if self._depth_data is None:
                raise RuntimeError("No depth frame received yet")
            depth_raw = self._depth_data
            dw = self._depth_width
            dh = self._depth_height

        # Depth camera intrinsics (87° HFOV)
        fx_d = dw / (2.0 * math.tan(DEPTH_HFOV / 2.0))
        fy_d = fx_d
        cx_d = dw / 2.0
        cy_d = dh / 2.0

        # Color camera intrinsics (69° HFOV)
        fx_c = cw / (2.0 * math.tan(COLOR_HFOV / 2.0))
        fy_c = fx_c
        cx_c = cw / 2.0
        cy_c = ch / 2.0

        depth = np.frombuffer(depth_raw, dtype=np.float32).reshape(dh, dw)
        color = np.frombuffer(color_raw, dtype=np.uint8).reshape(ch, cw, 3)

        # Pixel coordinate grids for depth image
        u_d = np.arange(dw, dtype=np.float32)
        v_d = np.arange(dh, dtype=np.float32)
        u_grid, v_grid = np.meshgrid(u_d, v_d)

        # Valid depth mask (within D435 range)
        valid_depth = (depth > 0.105) & np.isfinite(depth) & (depth < 10.0)

        z = depth[valid_depth]
        x = (u_grid[valid_depth] - cx_d) * z / fx_d
        y = (v_grid[valid_depth] - cy_d) * z / fy_d

        # Project 3D points into color camera frame to get RGB
        # (both sensors co-located on d435_link, so no translation needed)
        u_c = (x * fx_c / z + cx_c).astype(np.int32)
        v_c = (y * fy_c / z + cy_c).astype(np.int32)

        # Only keep points that project within the color image
        in_color = (u_c >= 0) & (u_c < cw) & (v_c >= 0) & (v_c < ch)

        x = x[in_color]
        y = y[in_color]
        z = z[in_color]
        u_c = u_c[in_color]
        v_c = v_c[in_color]

        # Sample RGB from color image at projected coordinates
        r = color[v_c, u_c, 0].astype(np.uint32)
        g = color[v_c, u_c, 1].astype(np.uint32)
        b = color[v_c, u_c, 2].astype(np.uint32)
        rgb = (r << 16) | (g << 8) | b

        num_points = int(len(z))

        # PCD header (matches real realsense module format)
        header = (
            f"VERSION .7\n"
            f"FIELDS x y z rgb\n"
            f"SIZE 4 4 4 4\n"
            f"TYPE F F F U\n"
            f"COUNT 1 1 1 1\n"
            f"WIDTH {num_points}\n"
            f"HEIGHT 1\n"
            f"VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {num_points}\n"
            f"DATA binary\n"
        ).encode("ascii")

        # Pack points as struct array: float32 x, y, z + uint32 rgb
        points = np.empty(num_points, dtype=[
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
        points['x'] = x
        points['y'] = y
        points['z'] = z
        points['rgb'] = rgb

        pcd_data = header + points.tobytes()

        if len(pcd_data) > MAX_GRPC_MESSAGE_SIZE:
            raise RuntimeError(
                f"Point cloud ({len(pcd_data)} bytes) exceeds 32MB gRPC limit")

        return pcd_data, "pointcloud/pcd"

    async def get_properties(
        self,
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Camera.Properties:
        # Return intrinsics of the main sensor (first in sensors list)
        main = self._sensors[0] if self._sensors else "color"
        hfov = COLOR_HFOV if main == "color" else DEPTH_HFOV
        fx = self._width / (2.0 * math.tan(hfov / 2.0))

        return Camera.Properties(
            supports_pcd=("color" in self._sensors and "depth" in self._sensors),
            intrinsic_parameters=IntrinsicParameters(
                width_px=self._width,
                height_px=self._height,
                focal_x_px=fx,
                focal_y_px=fx,
                center_x_px=self._width / 2.0,
                center_y_px=self._height / 2.0,
            ),
            distortion_parameters=DistortionParameters(model=""),
        )

    async def do_command(
        self,
        command: Mapping[str, Any],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, Any]:
        return {}
