"""Viam module entry point for the Gazebo RGBD camera.

Registers the GzCamera model and starts the module server.
Run as: python3 -m gz_camera_module.main <socket_path>
"""

import asyncio
import sys

from viam.module.module import Module
from viam.components.camera import Camera

from .gz_camera import GzCamera


async def main(address: str):
    module = Module(address)
    module.add_model_from_registry(Camera.SUBTYPE, GzCamera.MODEL)
    await module.start()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 -m gz_camera_module.main <socket_path>", file=sys.stderr)
        sys.exit(1)

    asyncio.run(main(sys.argv[1]))
