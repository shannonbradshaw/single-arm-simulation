"""Gazebo RGBD Camera — Viam Camera component backed by gz-transport.

Subscribes to Gazebo Harmonic's rgbd_camera sensor topics and serves
images through the Viam Camera API. Designed to run inside the same
Docker container as the Gazebo simulation.

Attributes:
    topic_prefix (str): gz-transport topic prefix, e.g. "/d435"
    width (int): expected image width (default 1280)
    height (int): expected image height (default 720)
"""

import io
import struct
import threading
from typing import Any, ClassVar, Dict, Mapping, Optional, Tuple, Union

from PIL import Image as PILImage

from viam.components.camera import Camera, DistortionParameters, IntrinsicParameters
from viam.media.video import CameraMimeType, RawImage
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.resource.base import ResourceBase
from viam.resource.types import Model, ModelFamily

from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node


class GzCamera(Camera, Reconfigurable):
    MODEL: ClassVar[Model] = Model(
        ModelFamily("viam", "camera"), "gazebo-rgbd"
    )

    def __init__(self, name: str):
        super().__init__(name)
        self._node: Optional[Node] = None
        self._topic_prefix = "/d435"
        self._width = 1280
        self._height = 720

        self._color_lock = threading.Lock()
        self._color_data: Optional[bytes] = None
        self._color_width = 0
        self._color_height = 0
        self._color_format = 0

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
        tp = attrs.get("topic_prefix")
        self._topic_prefix = tp.string_value if tp else "/d435"

        w = attrs.get("width")
        self._width = int(w.number_value) if w else 1280
        h = attrs.get("height")
        self._height = int(h.number_value) if h else 720

        self._setup_subscriptions()

    def _setup_subscriptions(self):
        self._node = Node()

        color_topic = f"{self._topic_prefix}/image"
        ok = self._node.subscribe(GzImage, color_topic, self._on_color)
        print(f"[gz-camera] Subscribe {color_topic}: {'OK' if ok else 'FAILED'}",
              flush=True)

        depth_topic = f"{self._topic_prefix}/depth_image"
        ok = self._node.subscribe(GzImage, depth_topic, self._on_depth)
        print(f"[gz-camera] Subscribe {depth_topic}: {'OK' if ok else 'FAILED'}",
              flush=True)

    def _on_color(self, msg: GzImage):
        with self._color_lock:
            self._color_data = msg.data
            self._color_width = msg.width
            self._color_height = msg.height
            self._color_format = msg.pixel_format_type

    def _on_depth(self, msg: GzImage):
        with self._depth_lock:
            self._depth_data = msg.data
            self._depth_width = msg.width
            self._depth_height = msg.height

    def _get_color_pil(self) -> PILImage.Image:
        """Convert latest color frame to a PIL Image."""
        with self._color_lock:
            if self._color_data is None:
                raise RuntimeError("No color frame received yet")
            data = self._color_data
            w = self._color_width
            h = self._color_height

        return PILImage.frombytes("RGB", (w, h), data)

    async def get_image(
        self,
        mime_type: str = "",
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Union[PILImage.Image, RawImage]:
        if mime_type and "depth" in mime_type:
            depth_bytes = self._depth_to_viam_format()
            return RawImage(data=depth_bytes, mime_type="image/vnd.viam.dep")

        return self._get_color_pil()

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
            if depth_m <= 0 or depth_m != depth_m:  # NaN or invalid
                mm = 0
            else:
                mm = min(65535, int(depth_m * 1000))
            struct.pack_into(">H", pixels, i * 2, mm)

        return header + bytes(pixels)

    async def get_point_cloud(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Tuple[bytes, str]:
        raise NotImplementedError("Point cloud not yet implemented")

    async def get_properties(
        self,
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Camera.Properties:
        return Camera.Properties(
            supports_pcd=False,
            intrinsic_parameters=IntrinsicParameters(
                width_px=self._width,
                height_px=self._height,
                focal_x_px=self._width / (2 * 0.9326),  # from hfov=87deg
                focal_y_px=self._width / (2 * 0.9326),
                center_x_px=self._width / 2.0,
                center_y_px=self._height / 2.0,
            ),
            distortion_parameters=DistortionParameters(model="no_distortion"),
        )

    async def do_command(
        self,
        command: Mapping[str, Any],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, Any]:
        return {}
