"""Minimal web viewer for Gazebo camera topics via MJPEG streaming."""

import threading
import time

from flask import Flask, Response
from gi.repository import GLib

from gz.msgs10.image_pb2 import Image as GzImage
from gz.transport13 import Node

app = Flask(__name__)

# Latest frame storage
latest_frames = {}
frame_locks = {}

CAMERA_TOPIC = "/overview_camera"


def on_image(msg):
    """Callback for Gazebo camera image messages."""
    try:
        # RGB_INT8 = 3 in gz.msgs PixelFormatType enum
        if msg.pixel_format_type == 3:
            from PIL import Image as PILImage
            import io

            img = PILImage.frombytes("RGB", (msg.width, msg.height), msg.data)
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=80)
            frame_data = buf.getvalue()

            with frame_locks[CAMERA_TOPIC]:
                latest_frames[CAMERA_TOPIC] = frame_data
    except Exception as e:
        print(f"Frame error: {e}")


def subscribe_to_camera():
    """Subscribe to the Gazebo camera topic."""
    latest_frames[CAMERA_TOPIC] = None
    frame_locks[CAMERA_TOPIC] = threading.Lock()

    node = Node()
    if node.subscribe(GzImage, CAMERA_TOPIC, on_image):
        print(f"Subscribed to {CAMERA_TOPIC}")
    else:
        print(f"Failed to subscribe to {CAMERA_TOPIC}")

    # Keep the gz-transport node alive
    while True:
        time.sleep(1.0)


def generate_mjpeg():
    """Generate MJPEG stream from latest frames."""
    while True:
        frame = None
        with frame_locks[CAMERA_TOPIC]:
            frame = latest_frames.get(CAMERA_TOPIC)

        if frame:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
        time.sleep(1.0 / 15)  # ~15 fps


@app.route("/")
def index():
    return """<!DOCTYPE html>
<html>
<head>
    <title>xArm 6 Simulation</title>
    <style>
        body { background: #1a1a1a; color: #fff; font-family: sans-serif;
               display: flex; flex-direction: column; align-items: center;
               margin: 0; padding: 20px; }
        h1 { font-weight: 300; margin-bottom: 10px; }
        img { max-width: 100%; border-radius: 4px;
              box-shadow: 0 4px 20px rgba(0,0,0,0.5); }
        .info { color: #888; font-size: 13px; margin-top: 10px; }
    </style>
</head>
<body>
    <h1>xArm 6 Simulation</h1>
    <img src="/stream" />
    <p class="info">Overview camera — 800x600 @ 15fps</p>
</body>
</html>"""


@app.route("/stream")
def stream():
    return Response(
        generate_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/health")
def health():
    has_frame = latest_frames.get(CAMERA_TOPIC) is not None
    return {"status": "ok", "receiving_frames": has_frame}


if __name__ == "__main__":
    # Start camera subscriber in background thread
    cam_thread = threading.Thread(target=subscribe_to_camera, daemon=True)
    cam_thread.start()

    print("Web viewer starting on http://0.0.0.0:8081")
    app.run(host="0.0.0.0", port=8081, threaded=True)
