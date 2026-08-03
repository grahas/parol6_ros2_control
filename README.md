# parol6_ros2_control

`ros2_control` integration for the [PAROL6](https://source-robotics.gitbook.io/parol6-guide) robot arm, driving it through the existing [parol6-server](https://github.com/PCrnjak/PAROL6-python-API) rather than reimplementing motion control.

**Targets a "lyrical"/6.7.x-generation `hardware_interface` API** (verified on Ubuntu 26.04 with `ros-lyrical-*` packages, on the machine actually connected to the robot). This is a newer, breaking-API generation than ROS2 Humble — see "hardware_interface API generations" below if you're on Humble or need to support both.

Three packages:

- **`parol6_hardware_interface`** — a `hardware_interface::SystemInterface` plugin (`Parol6SystemInterface`) for `ros2_control`: position command interface + position/velocity state interfaces per joint (`L1`-`L6`), driven via streaming `servo_j()` calls.
- **`parol6_bridge`** — a small Python asyncio daemon that speaks parol6-server's UDP wire protocol (via the `parol6` SDK's `AsyncRobotClient`) on one side, and a tiny fixed-size binary protocol over a local TCP socket on the other. Versioned together with `parol6_hardware_interface` since the loopback protocol between them is hand-synced, not code-generated.
- **`parol6_bringup`** — a minimal (no MoveIt) launch/config package: URDF splice, `ros2_controllers`-style YAML, one launch file. Written for machines/distros where MoveIt2 isn't packaged yet (see below) — if you have MoveIt2, prefer wiring into `parol6_moveit` instead (see "Wiring into parol6_moveit").

## Why the bridge

`ros2_control` hardware interfaces are C++ and run inside a real-time control loop. `parol6-server`'s wire protocol (msgpack framing, multicast status broadcasts, ack policies, IK-aware streaming commands, ...) is only implemented in the `parol6` Python SDK (`parol6/protocol/wire.py`, ~1900 lines as of `0.3.2`). Reimplementing that in C++ would be a lot of surface area to keep in sync with upstream, so `parol6_bridge` reuses it directly instead:

```
ros2_control (C++, real-time loop)        parol6_bridge (Python daemon)         parol6-server
Parol6SystemInterface   <--TCP loopback-->   uses parol6.AsyncRobotClient  <--UDP-->   (existing)
```

The loopback protocol is documented in `parol6_bridge/parol6_bridge/protocol.py` and mirrored by hand in `parol6_hardware_interface/include/parol6_hardware_interface/bridge_protocol.hpp` — both files say so in a comment; if you change one, change the other.

## hardware_interface API generations

This plugin was originally written against Humble's `hardware_interface` (manual `export_state_interfaces()`/`export_command_interfaces()` returning raw-pointer-backed `StateInterface`/`CommandInterface` vectors, `on_init(const HardwareInfo &)`). It was later ported to a newer generation (seen on a ROS2 distro called "lyrical", `ros2_control` `6.7.x`) with a materially different plugin-author API:

- `on_init` takes `const hardware_interface::HardwareComponentInterfaceParams &` (a wrapper around `HardwareInfo` plus an `rclcpp::Executor::WeakPtr`), not `HardwareInfo` directly.
- State/command interface *storage* is now owned and auto-exported by the framework from the URDF-declared interfaces; plugins no longer construct `StateInterface`/`CommandInterface` objects themselves. `export_state_interfaces()`/`export_command_interfaces()` are deprecated in favor of `on_export_state_interfaces()`/`on_export_command_interfaces()`, and the default implementations (which we rely on — we don't override either) build interfaces straight from the URDF.
- Plugins read/write through `get_state<T>(name)`/`set_state<T>(name, value)`/`get_command<T>(name)`/`set_command<T>(name, value)` (or, for repeated hot-path access, cached `StateInterface::SharedPtr`/`CommandInterface::SharedPtr` handles from `get_state_interface_handle()`/`get_command_interface_handle()`, resolved once in `on_configure()` — that's what this plugin does).

**These two generations are not source-compatible.** The code currently in this repo targets the newer generation only. If you need Humble, you'd fork at the commit before this port and maintain it separately, or add `#if`-based dual compilation (not attempted here — the two `on_init` signatures alone make it more than a preprocessor-guard job; a cleaner split would be a `-DPAROL6_HW_IFACE_GENERATION=...` CMake option selecting between two `.cpp` files).

`CMakeLists.txt` also changed: `ament_target_dependencies()` is gone in this generation (`target_link_libraries(... PUBLIC hardware_interface::hardware_interface ...)` with modern CMake imported targets is required instead) — if you're porting back to Humble, that macro still exists there and either form works.

## Fitting into a workspace

This repo is meant to sit inside a colcon workspace's `src/`. If MoveIt2 is available for your distro, pair it with [PAROL6-ROS2-MOVEIT](https://github.com/PCrnjak/PAROL6-ROS2-MOVEIT)'s `parol6`/`parol6_moveit` packages (see "Wiring into parol6_moveit"); otherwise pair it with just `parol6` (the description-only package from that same repo) and use `parol6_bringup` here instead.

```
ros2_ws/src/
  PAROL6-ROS2-MOVEIT/...        # upstream: parol6 (description) [+ parol6_moveit (MoveIt config), if available]
  parol6_ros2_control/          # this repo
    parol6_bridge/
    parol6_hardware_interface/
    parol6_bringup/             # no-MoveIt bringup; skip if using parol6_moveit instead
```

### Prerequisites

```bash
# substitute your distro name for "lyrical" (e.g. humble, jazzy)
sudo apt install ros-lyrical-ros2-control ros-lyrical-ros2-controllers \
                  ros-lyrical-joint-state-broadcaster ros-lyrical-joint-trajectory-controller \
                  ros-lyrical-xacro ros-lyrical-robot-state-publisher

# parol6_bridge needs the parol6 pip package importable by whichever python3 runs it:
pip install "git+https://github.com/PCrnjak/PAROL6-python-API.git@0.4.0"
```

`parol6` (via its `pinokin` dependency) currently ships prebuilt wheels requiring glibc >= 2.39. On Ubuntu 22.04 (glibc 2.35) this means `parol6`/`parol6_bridge` can't run natively there. Options, in order of preference:
- Run on a newer base (Ubuntu 24.04+/glibc 2.39+, where the prebuilt `pinokin` wheel just works) — this is what the "lyrical" verification above used.
- Run `parol6-server` + `parol6_bridge` on a separate machine/venv where `parol6` installs cleanly, and point `parol6_hardware_interface`'s `bridge_host` param at it over the network. `127.0.0.1` crosses transparently between WSL2 (mirrored networking mode) and its Windows host, which is how the Humble-generation prototype was validated.
- Build `pinokin` from source for your glibc.

If `parol6_bridge` needs to run under a specific venv (rather than whatever `python3` your ROS install uses), `pip install -e parol6_bridge/` into that venv, then point `parol6_bringup`'s `bridge_python` launch arg (or your own launch file) at that venv's `python3` — see `parol6_bringup/launch/parol6_control.launch.py`. `parol6_bridge` is invoked as `python3 -m parol6_bridge.bridge_node` rather than via its installed console-script entry point, specifically so this is possible; `launch`'s `FindExecutable` only does a literal `$PATH` search and can't resolve `ros2 run`-style ament-index executables anyway.

### Wiring into parol6_moveit

If your distro has MoveIt2 packaged: in `parol6_moveit/config/parol6.ros2_control.xacro`, the `parol6_ros2_control` macro needs `use_real_hardware` (default `false`, preserving the stock mock-hardware behavior), `bridge_host`, and `bridge_port` params, switching the `<plugin>` between `mock_components/GenericSystem` and `parol6_hardware_interface/Parol6SystemInterface`. `parol6.urdf.xacro` needs to declare and forward those three as xacro args. A `real_robot.launch.py` (mirroring `demo.launch.py` but starting `parol6_bridge` first and passing `use_real_hardware:=true` through `MoveItConfigsBuilder(...).robot_description(mappings={...})`) is the cleanest way to launch it. (Not included in this repo since it's a patch to someone else's generated package, not new code we own — reapply by hand, or fork `PAROL6-ROS2-MOVEIT` if you want it version-controlled too.) Verified working (state read path) on Humble; see "Known environment issues" for the controller-loading gotcha that also applies there.

If MoveIt2 isn't packaged for your distro yet, use `parol6_bringup` instead (below) — same hardware interface, no MoveIt/planning, just `ros2_control` + a `joint_trajectory_controller` you can send `FollowJointTrajectory` goals to directly.

### Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run (parol6_bringup, no MoveIt)

```bash
# 1. parol6-server -- must be either connected to real hardware, or explicitly
#    in simulator mode (parol6-server defaults to neither: with no --serial and
#    no simulator enabled, it accepts commands but never produces real motion
#    or state changes, silently. `RobotClient(...).simulator(True)` enables it
#    on a running server; --serial <port> connects to real hardware instead.)
parol6-server --log-level=INFO

# 2. ros2_control, from the ROS2 machine (starts parol6_bridge itself)
ros2 launch parol6_bringup parol6_control.launch.py \
    bridge_python:=/path/to/venv/with/parol6/bin/python3
# non-default parol6-server location:
#   ... robot_host:=192.168.1.50 robot_port:=5001

# 3. send it a trajectory (once parol6_arm_controller is 'active' --
#    ros2 control list_controllers to check)
ros2 action send_goal /parol6_arm_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [L1, L2, L3, L4, L5, L6]
  points:
  - positions: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start: {sec: 3, nanosec: 0}
"
```

## Known environment issues

- **`ros2` CLI hangs** (`ros2 node list`, `ros2 param get`, `ros2 control list_controllers` timing out or hanging indefinitely) **under WSL2**: FastDDS's multicast discovery misbehaves under WSL2's mirrored networking mode. Fix: `export ROS_LOCALHOST_ONLY=1` before sourcing any ROS setup, consistently across every terminal/process that needs to discover the others — a mismatch (some processes with it set, some without) looks like "sometimes it hangs" rather than a hard failure. Not observed on the non-WSL "lyrical" machine.

- **A controller requiring a non-empty parameter (e.g. `joint_trajectory_controller`'s `joints`) fails with `Length of parameter 'joints' is '0' but must be greater than '0'`, while `joint_state_broadcaster` (which tolerates an empty config) loads fine, using the identical YAML file:** this is **not** a `parol6_hardware_interface` bug, and (correcting an earlier note in this file) not necessarily an upstream `ros2_controllers` version-skew bug either — it's a real, fixable mechanism change in newer `ros2_control`. As of `ros2_control` `6.7.x` (source: `controller_manager.cpp`'s `determine_controller_node_options()`), `controller_manager` calls `use_global_arguments(false)` on every controller sub-node it spawns, meaning it **no longer automatically forwards its own `--params-file` arguments** to controllers. Each controller that needs parameters beyond its bare declaration must be given an explicit `<controller_name>.params_file` parameter, declared on `controller_manager` itself:
  ```python
  parameters=[
      controllers_yaml,                                       # controller_manager's own params + each controller's `type:`
      {"parol6_arm_controller.params_file": controllers_yaml},  # tells CM to forward this file to that controller's sub-node
  ]
  ```
  Diagnosed by: constructing a plain `rclpy.Node` named identically to the controller with the same `--params-file`, which correctly read the parameter (proving the YAML and rclcpp's file-to-node matching were both fine) — isolating the bug to `controller_manager`'s own sub-node construction — then reading `ros2_control`'s actual source rather than continuing to guess from log output. See `parol6_bringup/launch/parol6_control.launch.py` for the fix in context. If you hit this on Humble (an older generation where this shouldn't apply) via a hand-rolled launch file, audit for the same missing-forwarding pattern before assuming it's the same root cause -- it may be a different, Humble-specific issue instead.

- **`parol6-server` accepts and "executes" motion commands but position never changes, with no error anywhere**: check `RobotClient(...).is_simulator()` and `.ping().hardware_connected` -- if both are false/`False`, the server has neither real hardware nor its simulator enabled, and silently accepts commands into a state machine that never produces motion or feedback. `servo_j()` (which `parol6_bridge` uses) is explicitly fire-and-forget and won't surface this. Fix: `RobotClient(...).simulator(True)` for testing, or `--serial <port>` for real hardware.

- **`PAROL6-ROS2-MOVEIT`'s bundled URDF (`ros_parol/src/parol6/urdf/parol6.urdf`) has kinematics that don't match `PAROL6_ROBOT.py`** (the same pip package's own kinematic model, used by `parol6-server` itself for limits/FK/collision): several joints have opposite rotation axis signs, and L2/L3's valid ranges barely overlap between the two descriptions at all -- not a rounding issue, confirmed by direct forward-kinematics comparison (`robot.fkine(q)` vs. the URDF's TF chain for the same `q`, which produced unrelated poses before the fix). Fix upstreamed: [Source-Robotics/PAROL6-ROS2-MOVEIT#1](https://github.com/Source-Robotics/PAROL6-ROS2-MOVEIT/pull/1) -- replaces that URDF *and every mesh it references* with the pip package's own authoritative copies. **The mesh part matters as much as the URDF part**: this repo's `L1.STL`/`L2.STL`/etc. share filenames with the pip package's meshes but are *different files* (different `md5sum`, confirmed for every one of the 7 arm meshes) -- swapping only the URDF while keeping the old mesh files produces a visually-incoherent (but TF/limit-wise "correct") robot, which is exactly what happened on the first pass of this fix before it was caught by actually looking at it in RViz. Until the PR merges, apply the same patch locally: copy `parol6/urdf_model/urdf/PAROL6.urdf`, its `*.STL` visual meshes, *and* its `*_simplified.stl` collision meshes from the `parol6` pip package's install location over `ros_parol/src/parol6/urdf/parol6.urdf` and `.../meshes/` -- all three, not just the URDF.

- **Toggling a hardware component's lifecycle state (e.g. `ros2 control set_hardware_component_state Parol6System inactive` to safely drive the robot directly for testing, bypassing `ros2_control`) does not stop an already-`active` controller from fighting you.** `parol6_arm_controller` keeps holding its own last commanded setpoint the whole time the hardware is inactive; the moment the hardware goes back to `active` and its command interfaces become claimable again, the controller immediately re-asserts that stale setpoint, silently overriding whatever `on_activate()` just seeded and driving the arm back to wherever it was before you touched it -- easy to mistake for a bug in `Parol6SystemInterface` (it looks exactly like state being lost), but the state read path is fine; it's the *controller* that never got told anything changed. Fix: don't fight it -- either deactivate the controller too before manually moving the robot, or (simpler, and the actually-correct way to move the robot in the first place) send the desired pose as a normal trajectory goal through the controller's action interface, exactly as it's meant to be driven.

## Safety notes

- `parol6-server` has no authentication — `parol6_bridge`'s TCP socket likewise has none. Both default to `127.0.0.1`; don't rebind either to a non-loopback address unless the network is trusted.
- The hardware interface commands *position* only, streamed via `servo_j()` every control cycle — there's no velocity/effort limiting beyond what `parol6-server` itself enforces. Keep the physical E-stop reachable.
- `Parol6SystemInterface::read()` logs and holds last-known state on a transient bridge/network hiccup rather than erroring the whole `controller_manager` out; a *sustained* disconnect shows up as frozen `/joint_states` and a `write()` that keeps failing (throttled warning in the `ros2_control_node` log), not a crash.
- Observed tracking lag / slow convergence when driving a `joint_trajectory_controller` through this plugin: the controller's own trajectory interpolation continuously feeds a moving setpoint into `servo_j()`, which does its *own* independent Ruckig-based smoothing server-side -- two independent smoothing layers chasing each other. Worth tuning `parol6_arm_controller`'s `open_loop_control` / update rate against `parol6-server`'s own control loop rate together rather than assuming either default is right; not yet done here.

## License

MIT — see `LICENSE`.
