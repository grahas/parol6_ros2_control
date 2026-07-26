# parol6_ros2_control

`ros2_control` integration for the [PAROL6](https://source-robotics.gitbook.io/parol6-guide) robot arm, driving it through the existing [parol6-server](https://github.com/PCrnjak/PAROL6-python-API) rather than reimplementing motion control.

Two packages, versioned together because they're a matched pair — the local wire protocol between them is hand-synced, not code-generated:

- **`parol6_hardware_interface`** — a `hardware_interface::SystemInterface` plugin (`Parol6SystemInterface`) for `ros2_control`: position command interface + position/velocity state interfaces per joint (`L1`-`L6`), driven via streaming `servo_j()` calls.
- **`parol6_bridge`** — a small Python asyncio daemon that speaks parol6-server's UDP wire protocol (via the `parol6` SDK's `AsyncRobotClient`) on one side, and a tiny fixed-size binary protocol over a local TCP socket on the other.

## Why the bridge

`ros2_control` hardware interfaces are C++ and run inside a real-time control loop. `parol6-server`'s wire protocol (msgpack framing, multicast status broadcasts, ack policies, IK-aware streaming commands, ...) is only implemented in the `parol6` Python SDK (`parol6/protocol/wire.py`, ~1900 lines as of `0.3.2`). Reimplementing that in C++ would be a lot of surface area to keep in sync with upstream, so `parol6_bridge` reuses it directly instead:

```
ros2_control (C++, real-time loop)        parol6_bridge (Python daemon)         parol6-server
Parol6SystemInterface   <--TCP loopback-->   uses parol6.AsyncRobotClient  <--UDP-->   (existing)
```

The loopback protocol is documented in `parol6_bridge/parol6_bridge/protocol.py` and mirrored by hand in `parol6_hardware_interface/include/parol6_hardware_interface/bridge_protocol.hpp` — both files say so in a comment; if you change one, change the other.

## Fitting into a workspace

This repo is meant to sit inside a colcon workspace's `src/`, alongside a description/MoveIt package pair for PAROL6 — developed against [PAROL6-ROS2-MOVEIT](https://github.com/PCrnjak/PAROL6-ROS2-MOVEIT)'s `parol6`/`parol6_moveit` packages specifically. That repo's `parol6_moveit/config/parol6.ros2_control.xacro` needs a small patch to use `Parol6SystemInterface` instead of its default `mock_components/GenericSystem` — see "Wiring into parol6_moveit" below.

```
ros2_ws/src/
  PAROL6-ROS2-MOVEIT/...        # upstream: parol6 (description) + parol6_moveit (MoveIt config)
  parol6_ros2_control/          # this repo
    parol6_bridge/
    parol6_hardware_interface/
```

### Prerequisites

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
                  ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller

# parol6_bridge needs the parol6 pip package importable by whichever python3 runs it:
pip install "git+https://github.com/PCrnjak/PAROL6-python-API.git@0.3.2"
```

`parol6` (via its `pinokin` dependency) currently ships prebuilt wheels requiring glibc >= 2.39. On Ubuntu 22.04 (glibc 2.35) this means `parol6_bridge` can't run natively — either build `pinokin` from source, or run `parol6-server` + `parol6_bridge` on a machine/venv where `parol6` installs cleanly and point `parol6_hardware_interface`'s `bridge_host` param at it over the network. `127.0.0.1` crosses transparently between WSL2 (mirrored networking mode) and its Windows host, which is how this was validated during development.

### Wiring into parol6_moveit

In `parol6_moveit/config/parol6.ros2_control.xacro`, the `parol6_ros2_control` macro needs `use_real_hardware` (default `false`, preserving the stock mock-hardware behavior), `bridge_host`, and `bridge_port` params, switching the `<plugin>` between `mock_components/GenericSystem` and `parol6_hardware_interface/Parol6SystemInterface`. `parol6.urdf.xacro` needs to declare and forward those three as xacro args. A `real_robot.launch.py` (mirroring `demo.launch.py` but starting `parol6_bridge` first and passing `use_real_hardware:=true` through `MoveItConfigsBuilder(...).robot_description(mappings={...})`) is the cleanest way to launch it. (Not included in this repo since it's a patch to someone else's generated package, not new code we own — reapply by hand, or fork `PAROL6-ROS2-MOVEIT` if you want it version-controlled too.)

### Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run

```bash
# 1. parol6-server (wherever parol6 actually installs -- see prerequisites note above)
parol6-server --log-level=INFO

# 2. the bridge (same machine as parol6-server, or elsewhere reachable at --robot-host)
ros2 run parol6_bridge parol6_bridge --robot-host 127.0.0.1 --robot-port 5001 \
    --bind-host 127.0.0.1 --bind-port 6001

# 3. ros2_control + MoveIt, from the wsl/robot-side ROS2 machine
ros2 launch parol6_moveit real_robot.launch.py
```

## Known environment issues (Humble, 22.04, WSL2)

- **`ros2` CLI hangs** (`ros2 node list`, `ros2 param get`, `ros2 control list_controllers` timing out or hanging indefinitely): FastDDS's multicast discovery misbehaves under WSL2's mirrored networking mode. Fix: `export ROS_LOCALHOST_ONLY=1` before sourcing any ROS setup, consistently across every terminal/process that needs to discover the others — a mismatch (some processes with it set, some without) looks like "sometimes it hangs" rather than a hard failure.
- **`arm_group_controller` (or any controller requiring a non-empty `joints` param) fails to configure** with `'joints' parameter is empty` / `'command_interfaces' parameter is empty`, while `joint_state_broadcaster` configures fine: observed with `ros-humble-controller-manager` `2.54.0` against `ros-humble-ros2-controllers` `2.53.1` from the `packages.ros.org` mirror as of mid-2026. Reproduced identically with two structurally unrelated controller types (`joint_trajectory_controller` and `forward_command_controller`) and with `joint_trajectory_controller` rebuilt from source against the installed `2.54.0` headers, which ruled out YAML content, DDS flakiness, and controller-specific bugs. Looks like an upstream regression in that `controller-manager` build; not something fixed from this repo. `parol6_hardware_interface` itself was verified working end-to-end up to this point — `on_configure()`/`on_activate()` against a real bridge+server, and `joint_state_broadcaster` correctly relaying live joint state through it into `/joint_states` — the failure is entirely in the stock controller loading path, not the hardware interface.

## Safety notes

- `parol6-server` has no authentication — `parol6_bridge`'s TCP socket likewise has none. Both default to `127.0.0.1`; don't rebind either to a non-loopback address unless the network is trusted.
- The hardware interface commands *position* only, streamed via `servo_j()` every control cycle — there's no velocity/effort limiting beyond what `parol6-server` itself enforces. Keep the physical E-stop reachable.
- `Parol6SystemInterface::read()` logs and holds last-known state on a transient bridge/network hiccup rather than erroring the whole `controller_manager` out; a *sustained* disconnect shows up as frozen `/joint_states` and a `write()` that keeps failing (throttled warning in the `ros2_control_node` log), not a crash.

## License

MIT — see `LICENSE`.
