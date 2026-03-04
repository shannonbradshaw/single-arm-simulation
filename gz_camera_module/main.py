"""Viam module entry point for the Gazebo RGBD camera.

Patches the SDK's CameraRPCService.GetImages to set the proto Format enum
field (needed by viam-server's Go code), then registers GzCamera and starts.
Run as: python3 -m gz_camera_module.main <socket_path>
"""

import asyncio
import sys

from grpclib.server import Stream

from viam.module.module import Module
from viam.components.camera import Camera
from viam.components.camera.service import CameraRPCService
from viam.proto.component.camera import (
    GetImagesRequest,
    GetImagesResponse,
    Format,
    Image as ImageProto,
)
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.utils import struct_to_dict

from .gz_camera import GzCamera

# Map CameraMimeType string values to proto Format enum
_MIME_TO_FORMAT = {
    "image/jpeg": Format.FORMAT_JPEG,
    "image/png": Format.FORMAT_PNG,
    "image/vnd.viam.dep": Format.FORMAT_RAW_DEPTH,
    "image/vnd.viam.rgba": Format.FORMAT_RAW_RGBA,
}


async def _patched_get_images(self, stream: Stream[GetImagesRequest, GetImagesResponse]) -> None:
    """GetImages handler that sets the proto format field correctly."""
    request = await stream.recv_message()
    assert request is not None
    camera = self.get_resource(request.name)

    timeout = stream.deadline.time_remaining() if stream.deadline else None
    images, metadata = await camera.get_images(
        timeout=timeout,
        metadata=stream.metadata,
        extra=struct_to_dict(request.extra),
        filter_source_names=request.filter_source_names,
    )

    img_protos = []
    for img in images:
        mime_str = img.mime_type if isinstance(img.mime_type, str) else img.mime_type.value
        fmt = _MIME_TO_FORMAT.get(mime_str, Format.FORMAT_UNSPECIFIED)
        img_protos.append(ImageProto(
            source_name=img.name,
            format=fmt,
            image=img.data,
        ))

    response = GetImagesResponse(images=img_protos, response_metadata=metadata)
    await stream.send_message(response)


# Patch the SDK's service handler
CameraRPCService.GetImages = _patched_get_images

# Register the model creator
Registry.register_resource_creator(
    Camera.API,
    GzCamera.MODEL,
    ResourceCreatorRegistration(GzCamera.new),
)


async def main(address: str):
    module = Module(address)
    module.add_model_from_registry(Camera.API, GzCamera.MODEL)
    await module.start()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 -m gz_camera_module.main <socket_path>", file=sys.stderr)
        sys.exit(1)

    asyncio.run(main(sys.argv[1]))
