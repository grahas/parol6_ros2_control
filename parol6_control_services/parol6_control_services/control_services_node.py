"""Exposes one-off imperative PAROL6 operations -- home, gripper open/close
-- as std_srvs/Trigger ROS 2 services, instead of the ad-hoc standalone
Python scripts these previously required.

Deliberately NOT routed through ros2_control/parol6_hardware_interface:
that C++ plugin's real-time read/write loop and parol6_bridge's fixed
OP_UPDATE/OP_ENABLE/OP_DISABLE/OP_PING protocol (see parol6_bridge/
protocol.py) are built for continuous joint-position streaming, not one-off
commands -- home() in particular can block for tens of seconds, which has
no sane place in a real-time control loop. This node instead opens its own
independent parol6 RobotClient straight to parol6-server (same pattern
already used by parol6_dispenser's gripper/dispense I/O), separate from
whatever ros2_control connection may also be active via parol6_bridge.

IMPORTANT: robot_host/robot_port here must point at parol6-server itself
(default 127.0.0.1:5001) -- NOT at parol6_bridge's port (default 6001).
parol6_bridge speaks a private protocol with no write_io/home concept; a
RobotClient pointed at it will silently do nothing (see pick_and_place.py's
module docstring for how this was found).

Services (all std_srvs/srv/Trigger):
    /parol6/home            -- runs the homing sequence, blocks until done
    /parol6/gripper_open    -- write_io(gripper_io_pin, 1)
    /parol6/gripper_close   -- write_io(gripper_io_pin, 0)

Usage (after sourcing the workspace, with parol6-server already running):
    ros2 run parol6_control_services control_services_node
    ros2 service call /parol6/home std_srvs/srv/Trigger {}
"""

from __future__ import annotations

from parol6 import RobotClient
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class Parol6ControlServices(Node):
    def __init__(self):
        super().__init__("parol6_control_services")
        self.declare_parameter("robot_host", "127.0.0.1")
        self.declare_parameter("robot_port", 5001)
        self.declare_parameter("gripper_io_pin", 0)

        robot_host = self.get_parameter("robot_host").value
        robot_port = self.get_parameter("robot_port").value
        self._gripper_io_pin = self.get_parameter("gripper_io_pin").value

        self.get_logger().info(f"Connecting to parol6-server at {robot_host}:{robot_port}...")
        self._client = RobotClient(host=robot_host, port=robot_port)

        self.create_service(Trigger, "~/home", self._on_home)
        self.create_service(Trigger, "~/gripper_open", self._on_gripper_open)
        self.create_service(Trigger, "~/gripper_close", self._on_gripper_close)
        self.get_logger().info("Services ready: ~/home, ~/gripper_open, ~/gripper_close")

    def _on_home(self, request, response):
        try:
            self.get_logger().info("Homing...")
            self._client.home()
            self._client.wait_motion()
            angles = self._client.angles()
            response.success = True
            response.message = f"Homed. angles={angles}"
        except Exception as e:
            self.get_logger().error(f"Home failed: {e}")
            response.success = False
            response.message = str(e)
        return response

    def _on_gripper_open(self, request, response):
        return self._write_gripper(1, response)

    def _on_gripper_close(self, request, response):
        return self._write_gripper(0, response)

    def _write_gripper(self, value, response):
        try:
            self._client.write_io(self._gripper_io_pin, value)
            response.success = True
            response.message = f"write_io({self._gripper_io_pin}, {value})"
        except Exception as e:
            self.get_logger().error(f"Gripper write_io failed: {e}")
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        self._client.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = Parol6ControlServices()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
