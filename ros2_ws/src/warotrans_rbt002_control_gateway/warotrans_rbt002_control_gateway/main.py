"""Entry point: rclpy executor on a background thread, uvicorn (FastAPI) on the main thread."""

from __future__ import annotations

import logging
import os
import threading

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.executors import MultiThreadedExecutor

from .app import create_app
from .bridge import RosBridge


def _default_web_dist() -> str:
    try:
        return os.path.join(get_package_share_directory("warotrans_rbt002_control_gateway"), "web")
    except PackageNotFoundError:
        return ""


def main(args=None) -> None:
    logging.basicConfig(level=logging.INFO, format="[control_gateway] %(levelname)s %(message)s")
    rclpy.init(args=args)
    bridge = RosBridge()
    bridge.declare_parameter("host", "0.0.0.0")
    bridge.declare_parameter("port", 8080)
    bridge.declare_parameter("web_dist", "")
    bridge.declare_parameter("deadman_s", 0.3)
    g = lambda n: bridge.get_parameter(n).value  # noqa: E731
    token = os.environ.get("WARO_UI_TOKEN") or None
    if token is None:
        bridge.get_logger().warning("WARO_UI_TOKEN not set: anyone on this network can open the control UI")
    web_dist = str(g("web_dist")) or _default_web_dist()
    if not os.path.isdir(web_dist):
        bridge.get_logger().warning(f"web UI not found at {web_dist!r}: API/WebSocket only")

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    spin = threading.Thread(target=executor.spin, name="ros_spin", daemon=True)
    spin.start()

    import uvicorn

    app = create_app(bridge, web_dist=web_dist, token=token, deadman_s=float(g("deadman_s")))
    try:
        uvicorn.run(app, host=str(g("host")), port=int(g("port")), log_level="info")
    finally:
        try:
            bridge.publish_twist(0.0, 0.0)
        except Exception:
            pass
        executor.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
