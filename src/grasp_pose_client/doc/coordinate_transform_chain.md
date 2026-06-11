# Coordinate Transform Chain: VLA Server → `/commanded_pose`

This document focuses on every coordinate transformation step that `grasp_pose_client_node` performs after the VLA server returns the best grasp pose, up to publishing a `PoseStamped` on the `/commanded_pose` topic.

---

## Overall flow

```
VLA Server (HTTP)
  └─ returns grasp pose (camera optical frame)
       │
       ▼
Step 1  Parse JSON → build T_grasp_cam (4×4, SE3)
       │
       ▼
Step 2  TF2 lookup: LIO_base_link ← camera_color_optical_frame
        → yields T_base_camera (4×4)
       │
       ▼
Step 3  Matrix multiplication: T_grasp_base = T_base_camera × T_grasp_cam
       │
       ▼
Step 4  Apply hand-eye calibration residual correction: p_final = p_grasp_base + grasp_offset_base_xyz
       │
       ▼
Step 5  Axis-convention remap: R_tcp = R_grasp_base @ GRASP_TO_TCP_AXES
        grasp (X=closing, Y=lateral, Z=approach) → LIO TCP (X=approach, Y=closing, Z=lateral)
       │
       ▼
Step 6  Rotation matrix → quaternion (Shepperd's method)
       │
       ▼
Step 7  Pack into geometry_msgs/PoseStamped
        frame_id = "LIO_base_link"
       │
       ├─ publish → ~/best_grasp      (latched, for RViz)
       ├─ publish → ~/grasps          (latched, all Top-K)
       └─ publish → /commanded_pose   (consumed by downstream execution)
```

---

## Detailed steps

### Step 1 — Parse the server response and build T_grasp_cam

Code location: [`_transform_to_base()`](../grasp_pose_client/grasp_pose_client_node.py#L582)

Each grasp entry (dict) from the server comes in one of two formats:

| Field | Meaning |
|------|------|
| `pose_4x4` | 4×4 homogeneous transform matrix, used directly |
| `position_xyz` + `quaternion_xyzw` | Position vector + quaternion; the client builds the rotation matrix itself and assembles the 4×4 |

Quaternion → rotation matrix (when `pose_4x4` is absent):

```python
x, y, z, w = quat[0..3]
R = [
    [1-2(y²+z²),  2(xy-wz),   2(xz+wy)],
    [2(xy+wz),    1-2(x²+z²), 2(yz-wx)],
    [2(xz-wy),    2(yz+wx),   1-2(x²+y²)],
]
T_grasp_cam[:3,:3] = R
T_grasp_cam[:3, 3] = position_xyz
```

All coordinates are in **`camera_color_optical_frame`** (the RealSense color camera optical frame, Z axis pointing into the scene).

---

### Step 2 — TF2 lookup of T_base_camera

Code location: [`_lookup_T_base_camera()`](../grasp_pose_client/grasp_pose_client_node.py#L549)

```
lookup_transform(
    target_frame = "LIO_base_link",              # robot_base_frame_id (IK URDF root frame)
    source_frame = "camera_color_optical_frame", # gripper_frame_id
    stamp = color_msg.header.stamp,              # image timestamp (service-call path)
              or Time()                          # latest available (polling path)
)
```

TF chain (full path at runtime):

```
camera_color_optical_frame
  ← camera_color_frame          (RealSense driver, static)
    ← camera_link               (RealSense driver, static)
      ← lio_gripper_interface_link  (hand-eye calibration static TF, published by the launch file)
        ← lio_link6G / lio_link56 / ... / lio_link12
          ← LIO_robot_base_link ← LIO_base_link ← ... ← map
```

The static TF segment `lio_gripper_interface_link` → `camera_link` is published by the
`static_transform_publisher` node in the launch file
([grasp_pose_client.launch.py](../launch/grasp_pose_client.launch.py#L115)):

```
Hand-eye calibration result:
  translation: x=-0.10 m, y=0.0, z=+0.052 m
  rotation:    RPY(roll=0, pitch=-π/2, yaw=0)  →  Ry(-90°)
  quaternion:  qx=0, qy=-0.7071, qz=0, qw=0.7071
```

**Fallback on lookup failure**: if TF times out (default 0.2 s), `T_base_camera` returns `None`
and the pose is published directly in the camera frame (`frame_id = gripper_frame_id`), so
downstream receives untransformed coordinates.

---

### Step 3 — SE(3) matrix multiplication

Code location: [`_transform_to_base()` L611](../grasp_pose_client/grasp_pose_client_node.py#L611)

```python
T_grasp_base = T_base_camera @ T_grasp_cam   # (4×4) @ (4×4)
R_base = T_grasp_base[:3, :3]
t_base = T_grasp_base[:3, 3]
```

This is a standard rigid-body transform composition: it maps "grasp pose relative to the camera" directly to "grasp pose relative to the robot base".

---

### Step 4 — Hand-eye calibration residual correction

Code location: [`_transform_to_base()` L614](../grasp_pose_client/grasp_pose_client_node.py#L614)

```python
if offset_base is not None:
    t_base = t_base + offset_base   # offset_base = grasp_offset_base_xyz
```

The parameter `grasp_offset_base_xyz` (default `[0.0, 0.0, 0.0]`) absorbs systematic extrinsic-calibration bias.
Axis convention (`LIO_base_link` frame, i.e. the IK URDF root frame):

| Axis | Direction |
|----|------|
| +x | Forward reach direction of the arm |
| +y | Left |
| +z | Up |

> **Note**: after `robot_base_frame_id` changed from the old `LIO_robot_base_link` to `LIO_base_link`,
> the axis definitions changed accordingly (the two frames differ by Rz(90°)). Any historical offset
> calibration based on the old axis directions must be recalibrated.

---

### Step 5 — Axis-convention remap (grasp → LIO TCP)

Code location: [`_transform_to_base()`](../grasp_pose_client/grasp_pose_client_node.py) (`GRASP_TO_TCP_AXES`)

```python
R_base = T_grasp_base[:3, :3] @ GRASP_TO_TCP_AXES
```

The grasp pose and the LIO TCP frame (`lio_tcp_link`) use different axis semantics:

| Physical meaning | Grasp convention | LIO TCP convention |
|---|---|---|
| approach | Z | X |
| closing | X | Y |
| lateral | Y | Z |

`GRASP_TO_TCP_AXES = [[0,1,0],[0,0,1],[1,0,0]]` is the proper rotation (det = +1) that
re-expresses the grasp axes in the TCP convention, so the published quaternion can be
consumed directly as the `lio_tcp_link` target orientation by both the Pinocchio IK
feasibility check and the PANOC IK in `panda_ik_teleop`. Once the arm reaches the
target, the TF frame `lio_tcp_link` should coincide with `/commanded_pose` in RViz
(position and all three axes).

---

### Step 6 — Rotation matrix → quaternion

Code location: [`_transform_to_base()` L617–L649](../grasp_pose_client/grasp_pose_client_node.py#L617)

Uses **Shepperd's method** (a numerically stable four-branch algorithm that avoids division by values close to 0):

```
trace > 0     → standard formula
R[0,0] is max → pivot on qx
R[1,1] is max → pivot on qy
R[2,2] is max → pivot on qz
```

Normalization and hemisphere constraint:
```python
q = q / |q|          # L2 normalization
if q[3] < 0: q = -q  # enforce w ≥ 0 (unique representation)
```

Output format: `[qx, qy, qz, qw]` (ROS standard)

---

### Step 7 — Pack into PoseStamped and publish

Code locations: [`_build_pose_stamped()`](../grasp_pose_client/grasp_pose_client_node.py#L923),
[`_publish_visualisation()`](../grasp_pose_client/grasp_pose_client_node.py#L688)

```python
msg = PoseStamped()
msg.header.stamp    = stamp        # image timestamp (preserves timing)
msg.header.frame_id = "LIO_base_link"   # robot_base_frame_id = IK URDF root frame
msg.pose.position.{x,y,z}         = t_base
msg.pose.orientation.{x,y,z,w}    = q
```

Publishing logic (`_publish_visualisation`):

```python
best = response.grasps[0]          # Top-1 highest-scoring grasp
self._best_pose_pub.publish(best)  # ~/best_grasp  (latched)
self._commanded_pose_pub.publish(best)  # /commanded_pose
# All Top-K are also published to ~/grasps (PoseArray, latched)
```

**Only `grasps[0]` (the highest-scoring grasp) is sent to `/commanded_pose`.**

---

## Differences between the two trigger paths

| | Service-call path | Web UI polling path |
|---|---|---|
| Triggered by | ROS2 service call `~/request_grasp` | 0.5 Hz timer polling `/poll_publish` |
| TF timestamp | `color_msg.header.stamp` | `Time()` (latest available) |
| IK check | Optional (`ik_bypass=True` skips by default) | Same |
| Publish path | Identical: `_transform_to_base` → `_publish_visualisation` | Identical |

---

## Debug outputs

Besides `/commanded_pose`, the node publishes the following debug topics that can be overlaid in RViz for comparison:

| Topic | Content |
|-------|------|
| `~/best_grasp_camera` | Raw server pose in the camera frame (untransformed) |
| `~/grasps_camera` | All Top-K raw poses in the camera frame |
| TF frame `grasp_best` | TF with `LIO_base_link` as parent, refreshed at 10 Hz |
| TF frame `grasp_best_cam` | TF with `camera_color_optical_frame` as parent, refreshed at 10 Hz |

`grasp_best` uses the LIO TCP axis convention (X=approach, Y=closing, Z=lateral) while
`grasp_best_cam` keeps the raw grasp convention (X=closing, Z=approach), so their axes
are expected to differ by the fixed `GRASP_TO_TCP_AXES` permutation. The **positions**
of the two frames must still coincide in RViz; a position offset means the extrinsic
calibration has a residual, which can be tuned via `grasp_offset_base_xyz`. After
execution, `lio_tcp_link` should coincide with `grasp_best` in both position and axes.
