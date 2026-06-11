# `/commanded_pose` Consumption Chain

This document describes what happens after `grasp_pose_client` publishes a `PoseStamped` to `/commanded_pose`: how downstream nodes consume the message, solve for joint angles, and ultimately drive the robot arm.

> For the upstream publishing logic see [coordinate_transform_chain.md](coordinate_transform_chain.md).

---

## Overall pipeline

```
/commanded_pose  (PoseStamped, frame_id=LIO_base_link)
        │
        ▼
panda_ik_teleop node  [main_teleop.cpp]
  ├─ TF2 transform: LIO_base_link → LIO_robot_base_link (executor URDF root; z+0.2655 m, Rz+90°)
  └─ PANOC IK solve (Rust, k::Chain)
        │
        ▼
/panda_ik_teleop/output  (Float64MultiArray, 6× joint angles, radians)
        │
        ▼
ik_stream_to_action node  [ik_stream_to_action.py]
  ├─ radians → degrees
  ├─ rate limiting (≤2°/tick, 50 Hz)
  └─ send ExecuteFunction action goal
        │
        ▼
/execute_function  (fp_core_msgs/action/ExecuteFunction)
  action = "move_joints"
  joint_position = [deg×6]
        │
        ▼
Robot arm hardware
```

---

## Node 1: `panda_ik_teleop`

**Source file**: [cpp_src/main_teleop.cpp](../../panda-ik/cpp_src/main_teleop.cpp)  
**Node name**: `panda_ik_teleop`  
**Run rate**: 100 Hz main loop

### 1.1 Subscription to `/commanded_pose`

```cpp
commanded_pose_sub = node->create_subscription<PoseStamped>(
    "/commanded_pose", 1,
    [&](PoseStamped::SharedPtr msg) {
        // The executor URDF (lio_arm.urdf) is rooted at base_footprint, which
        // coincides with LIO_robot_base_link — so the pose is TF-transformed
        // from LIO_base_link into LIO_robot_base_link (z+0.2655 m, Rz+90°).
        PoseStamped transformed =
            tfBuffer.transform(*msg, "LIO_robot_base_link", tf2::durationFromSec(0.2));
        commandedPose.pose = transformed.pose;

        commandedVel = Twist();   // zero out velocity (static target)
        frame_id = "lio_tcp_joint";
        start = true;             // trigger the main loop
    });
```

**Note**: `/commanded_pose` is published in `LIO_base_link` (`robot_base_frame_id`), while the
Rust PANOC IK solves in the executor URDF root frame `base_footprint` ≡ `LIO_robot_base_link`.
The TF2 transform above bridges the two (they differ by z+0.2655 m and Rz+90°). On TF failure
the message is **rejected** (unless its frame_id is already `LIO_robot_base_link`) — executing
a pose in the wrong frame would send the arm to a rotated/offset target.

### 1.2 Main loop (100 Hz)

Each iteration processes the current `commandedPose`:

```
1. Velocity integration (teleop mode):
   position += vel * (1/freq)
   orientation = motion_rot * orientation  → normalize

2. Call the Rust IK solver:
   solve(joint_angles, "lio_tcp_joint",
         position, orientation, velocity, errors, w=1.0)
   → updates joint_angles[6] (radians)

3. Continuity check (after initialized):
   |joint_angles[i] - prev[i]| ≤ 0.1 rad  →  valid
   otherwise check whether the end-effector position is within ±5 cm

4. Publish /panda_ik_teleop/output (Float64MultiArray, 6×rad)
5. Publish panda_commanded_pose (PoseStamped, for RViz debugging)
```

---

## Node 1 core: Rust PANOC IK solver

**Source file**: [src/lib.rs](../../panda-ik/src/lib.rs)  
**Libraries**: `k` (forward kinematics) + `optimization_engine` (PANOC optimizer)

### IK problem definition

```
minimize   J(q) = 100·||p(q) - p_target||²
                + w·||angle(R(q), R_target)||²
                + movement_cost(q, q0)

subject to lb ≤ q ≤ ub
```

| Term | Meaning |
|----|------|
| `p(q)` | Cartesian position of `lio_tcp_joint` at joint angles `q` |
| `p_target` | Target position (from the TF-transformed pose) |
| `w·rotation_cost` | Orientation error (squared angle), weight `w=1.0` |
| `movement_cost` | Penalizes deviation from the initial state `q0`, encouraging small motions |

### Joint limits (radians)

| Joint | Lower | Upper |
|------|------|------|
| joint 1 | -2.793 | +2.793 |
| joint 2 | -1.745 | +1.745 |
| joint 3 | -1.745 | +1.745 |
| joint 4 | -2.793 | +2.793 |
| joint 5 | -1.745 | +1.745 |
| joint 6 | -2.793 | +2.793 |

(The actual values are shrunk by ±0.1 rad as a soft margin.)

### Solver parameters

| Parameter | Value | Meaning |
|------|----|------|
| Max iterations | 50 | PANOC inner iterations |
| Max solve time | 7 ms | Returns the current best on timeout |
| Gradient method | Finite differences | h = 1000·ε_f64 |
| PANOC cache | 6 DOF, tol=1e-6 | Global singleton |

### Error flags (`errors[4]`)

| Index | Meaning |
|------|------|
| `[0]` | Optimizer threw an exception → fall back to `joint_angles` |
| `[1]` | Did not converge (time or iteration limit exceeded) |
| `[2,3]` | Unused |

---

## Node 2: `ik_stream_to_action`

**Source file**: [lio_specific_pkg_ros2/ik_stream_to_action.py](../../lio_specific_pkg_ros2/lio_specific_pkg_ros2/ik_stream_to_action.py)  
**Node name**: `joint_io_client`

### Subscription to `/panda_ik/output` (or a configured topic)

```python
def _ik_cb(self, msg: Float64MultiArray):
    arm6_deg = [math.degrees(x) for x in msg.data[:6]]
    self._latest_ik_deg = arm6_deg   # cache only, do not send immediately
```

### Control loop (50 Hz timer)

```
1. Rate limiting:
   desired[i] = clamp(target[i], last[i] ± max_step_deg)
   default max_step_deg = 2.0°/tick

2. Deadband filtering:
   if |desired[i] - last_cmd[i]| < eps (0.02°): skip

3. Send ExecuteFunction action goal:
   action  = "move_joints"
   bridge  = "core"
   arguments = {
       "joints": [1,2,3,4,5,6],
       "joint_position": [deg×6]
   }
```

### Purpose of rate limiting

The 100 Hz IK can change joint angles by at most 2° per update, and the 50 Hz control loop keeps the send rate bounded, preventing abrupt commands from damaging the motors.

---

## Full data-flow table

| Stage | Topic/Interface | Message type | Frame/Unit |
|------|-----------|----------|---------|
| VLA Server → client | HTTP JSON | dict | `camera_color_optical_frame` |
| Client → IK node | `/commanded_pose` | `PoseStamped` | `LIO_base_link` |
| Inside IK node (after TF) | `commandedPose` (in memory) | `Pose` | `LIO_robot_base_link` |
| IK node → action bridge | `/panda_ik_teleop/output` | `Float64MultiArray` | radians × 6 |
| Action bridge → hardware | `/execute_function` | `ExecuteFunction` goal | degrees (°) × 6 |

---

## Differences between the two IK nodes

The project contains two IK node implementations:

| | `main_teleop.cpp` (currently used) | `main.cpp` (legacy) |
|--|-------------------------------|-------------------|
| Node name | `panda_ik_teleop` | `panda_ik` |
| `/commanded_pose` handling | **With TF transform** (→ `LIO_robot_base_link`) | No transform, used directly |
| Teleop velocity support | Yes (`/input` twist) | No |
| Direction command support | Yes (`/directions` string) | No |
| Trajectory following | Yes (`/simulator/end_effector_pose`) | No |
| `weighted_pose` mode | No | Yes (`/weighted_pose`) |

`main_teleop.cpp` is the version actually running today (see recent git commits).

---

## Key caveats

1. **Coordinate frames**: `robot_base_frame_id = "LIO_base_link"` is shared by the published poses and the Pinocchio feasibility check (`ik_base_link`, URDF `lio_arm_reframed.urdf`). The Rust PANOC IK uses a different URDF (`lio_arm.urdf`) rooted at `base_footprint` ≡ `LIO_robot_base_link`, so `panda_ik_teleop` TF-transforms incoming poses into `LIO_robot_base_link` before solving (z+0.2655 m, Rz+90°) and rejects the message if that transform fails.

2. **IK convergence time**: a single solve is capped at 7 ms (50 iterations). On non-convergence, `errors[1]=true` but the current best solution is still returned, so the control loop never blocks.

3. **Rate limiting**: the 2°/tick limit in `ik_stream_to_action` is the last line of defense, preventing large IK jumps from being sent straight to the hardware.

4. **Initial joint state**: `joint_angles` is hard-coded to `{-1.621, -1.197, 1.274, 0.025, 1.482, -0.057}` (rad) and updated in place after every successful IK solve, serving as the warm-start seed for the next solve.

5. **Axis convention of `grasp_offset_base_xyz`**: the offset is applied in the `LIO_base_link` frame (+x = forward reach, +y = left, +z = up). This differs from the previously documented `LIO_robot_base_link` frame (the two differ by Rz(90°)) — keep this in mind when recalibrating.

6. **TCP axis convention**: the quaternion in `/commanded_pose` is the target orientation of `lio_tcp_link`, whose convention is **X = approach (tool axis), Y = closing, Z = lateral** (from the URDF: `lio_tcp_joint` sits 0.189 m along gripper +X with `rpy=0`, and the fingers swing about Z). `grasp_pose_client` already remaps the grasp convention (X=closing, Z=approach) via `GRASP_TO_TCP_AXES` before publishing, so `panda_ik_teleop` consumes the quaternion as-is with no further correction. Verification in RViz: after the arm reaches the target, the TF frame `lio_tcp_link` must coincide with `/commanded_pose` in position and all three axes.
