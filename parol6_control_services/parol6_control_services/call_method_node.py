"""Generic pass-through node: exposes any parol6 RobotClient method as a
single ROS 2 service, so a new robot operation never needs a new hand-wired
service -- callers pick the method by name and pass JSON-encoded args.

Why this exists: parol6_control_services started as a handful of specific
Trigger services (home, gripper_open, gripper_close). Every new capability
needed (park, force-home, arbitrary jog, ...) meant writing, building, and
redeploying a new service. RobotClient already exposes the robot's full
command surface (move_j, move_l, jog_j, write_io, select_tool, ...); this
node just makes that whole surface reachable from ROS 2 directly, once.

Security note: only methods in ALLOWED_METHODS may be called -- derived
from RobotClient's own public API, minus a short denylist of methods that
would break this node's own connection (close) or return non-JSON-
serializable live objects with no useful remote meaning. There is no
sandboxing beyond that; anything on the allowlist can command the real
robot, same as parol6_control_services' existing services already can.

Service: /parol6_control_services/call_method (parol6_interfaces/srv/CallMethod)

Usage:
    ros2 service call /parol6_control_services/call_method parol6_interfaces/srv/CallMethod \
        "{method: 'home', args_json: '[]', kwargs_json: '{\"wait\": true, \"force\": true}'}"
    ros2 service call /parol6_control_services/call_method parol6_interfaces/srv/CallMethod \
        "{method: 'move_j', args_json: '[[90.0, -90.0, 180.0, 0.0, 0.0, 180.0]]', kwargs_json: '{\"duration\": 4.0, \"wait\": true}'}"
    ros2 service call /parol6_control_services/call_method parol6_interfaces/srv/CallMethod \
        "{method: 'angles', args_json: '[]', kwargs_json: '{}'}"
"""

from __future__ import annotations

import dataclasses
import json

from parol6 import RobotClient
from parol6_interfaces.srv import CallMethod
import rclpy
from rclpy.node import Node

# close() would tear down this node's own shared connection out from under
# it; excluded rather than left to fail confusingly on the next call.
_DENYLIST = {"close"}

ALLOWED_METHODS = frozenset(
    name
    for name in dir(RobotClient)
    if not name.startswith("_") and callable(getattr(RobotClient, name)) and name not in _DENYLIST
)


def _json_default(obj):
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return dataclasses.asdict(obj)
    return str(obj)


class CallMethodNode(Node):
    def __init__(self):
        super().__init__("parol6_call_method")
        self.declare_parameter("robot_host", "127.0.0.1")
        self.declare_parameter("robot_port", 5001)
        robot_host = self.get_parameter("robot_host").value
        robot_port = self.get_parameter("robot_port").value

        self.client = RobotClient(host=robot_host, port=robot_port)
        self.create_service(CallMethod, "~/call_method", self._on_call)
        self.get_logger().info(
            f"Ready: ~/call_method -- {len(ALLOWED_METHODS)} RobotClient methods allowed"
        )

    def _on_call(self, request, response):
        method = request.method
        if method not in ALLOWED_METHODS:
            response.success = False
            response.error = f"'{method}' is not an allowed method"
            return response

        try:
            args = json.loads(request.args_json) if request.args_json else []
            kwargs = json.loads(request.kwargs_json) if request.kwargs_json else {}
        except json.JSONDecodeError as e:
            response.success = False
            response.error = f"invalid JSON in args_json/kwargs_json: {e}"
            return response

        try:
            fn = getattr(self.client, method)
            result = fn(*args, **kwargs)
            response.success = True
            response.result_json = json.dumps(result, default=_json_default)
        except Exception as e:
            response.success = False
            response.error = f"{type(e).__name__}: {e}"
        return response

    def destroy_node(self):
        self.client.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CallMethodNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
