

# grasp_pose_client — Architecture & Data Flow

## Overview

`grasp_pose_client` is a ROS 2 node that bridges the robot and a remote VLM-based grasp server. It synchronises color, depth, and camera-info streams from a RealSense camera, encodes them, and POSTs them to an HTTP server that returns ranked 6-DOF grasp poses. The node transforms those poses from the camera optical frame into the robot base frame using TF2, optionally validates each candidate with a Pinocchio-based IK solver, and publishes the best pose to `/commanded_pose` for downstream execution. It also integrates with the server's Web UI via a 2 Hz polling loop.

**Key dependencies:** ROS 2 (rclpy, tf2, message_filters), OpenCV, NumPy, `requests`; Pinocchio is optional (enables IK filtering).

---

## 1. Node Architecture

```mermaid
graph TD
  subgraph Inputs
    A["/camera/color/image_raw\nsensor_msgs/Image"]
    B["/camera/aligned_depth_to_color/image_raw\nsensor_msgs/Image"]
    C["/camera/color/camera_info\nsensor_msgs/CameraInfo"]
    D["/joint_states\nsensor_msgs/JointState"]
  end

  subgraph GraspPoseClientNode
    SYNC["ApproximateTimeSynchronizer\nslop=0.05 s, queue=10"]
    CACHE["Latest Frame Cache\n_latest = color, depth, info, stamp"]
    SVC["~/request_grasp\nService (MutuallyExclusive)"]
    POLL["Poll Timer — 2 Hz\nWeb UI integration"]
    TF["TF2 Buffer\nbase ← camera lookup"]
    HTTP["HTTPClient\nPOST /grasp"]
    IK["Pinocchio IK\n(optional, 2-stage GN)"]
    TFB["TF Broadcaster — 10 Hz\ngrasp_best, grasp_best_cam"]
  end

  subgraph Outputs
    E["~/best_grasp\nPoseStamped (latched)"]
    F["~/grasps\nPoseArray (latched)"]
    G["/commanded_pose\nPoseStamped"]
    H["TF frame: grasp_best\n(child of robot_base_frame_id)"]
    I["~/best_grasp_camera\nPoseStamped (debug, camera frame)"]
    J["~/grasps_camera\nPoseArray (debug, camera frame)"]
  end

  A --> SYNC
  B --> SYNC
  C --> SYNC
  SYNC --> CACHE
  D --> IK

  SVC --> CACHE
  SVC --> TF
  SVC --> HTTP
  HTTP --> IK
  IK --> SVC

  SVC --> E
  SVC --> F
  SVC --> G
  SVC --> I
  SVC --> J
  TFB --> H
  POLL --> HTTP
```

---

## 2. Grasp Request — Sequence Diagram

This is the main path triggered when a caller invokes the `~/request_grasp` service.

```mermaid
sequenceDiagram
  participant Caller
  participant Node as GraspPoseClientNode
  participant TF as TF2 Buffer
  participant Server as Grasp HTTP Server
  participant IK as Pinocchio IK (optional)
  participant Pub as ROS Publishers

  Caller->>Node: ~/request_grasp {task_spec, top_k, num_candidates}

  Node->>Node: _take_snapshot()<br/>validate age < max_snapshot_age_s (2.0 s)

  Node->>Node: image_conversion:<br/>color → PNG bytes (cv2.imencode)<br/>depth → .npy float32 metres<br/>CameraInfo.K → 3×3 JSON

  Node->>TF: lookup_transform(robot_base_frame_id ← gripper_frame_id, stamp)
  TF-->>Node: T_base_camera (4×4) or None on timeout

  Node->>Server: POST /grasp {rgb.png, depth.npy, K, task_spec, top_k, ...}
  Server-->>Node: JSON {grasps[], scores[], widths[], run_id, frame_id}

  alt IK check enabled (ik_urdf_path set)
    Note over Node,IK: grasps are still in camera frame here
    Node->>IK: _check_ik_feasibility(grasps_cam, q_seed)<br/>internally: T_ik = TF(LIO_base_link ← camera_optical)<br/>p_ik = T_ik × T_grasp_cam  (in-memory only, not stored)
    IK-->>Node: passed_grasps[] (camera-frame dicts, unchanged)
    Node->>Server: POST /submit_ik_result {run_id, trace_id, grasps: passed[]}
    Note over Node,Server: server stores IK-passing camera-frame grasps;<br/>ranking happens later via /select_and_execute
  end

  Note over Node: execute path only — after /select_and_execute triggers mode=execute
  Node->>Node: _transform_to_base() for each grasp<br/>T_grasp_base = T_base_camera × T_grasp_cam<br/>+ grasp_offset_base_xyz

  Node->>Pub: ~/best_grasp  (grasps[0])
  Node->>Pub: ~/grasps       (all top-K)
  Node->>Pub: /commanded_pose (copy of grasps[0])
  Node->>Pub: ~/best_grasp_camera (debug — untransformed)
  Node->>Pub: ~/grasps_camera     (debug — untransformed)
  Node->>Node: _set_grasp_tfs() — stash for 10 Hz TF broadcaster

  Node-->>Caller: {success, grasps[], scores[], widths[], run_id, message}
```

---

## 3. Coordinate Transformation Pipeline

Grasp poses leave the server expressed in the camera optical frame. This section shows how they arrive in the robot base frame.

```mermaid
flowchart TD
  A["Server response\ngrasp pose T_grasp_cam ∈ SE(3)\nframe: camera_color_optical_frame"] --> B{TF lookup\nsuccessful?}

  B -- No --> C["Fallback: publish in camera frame\nframe_id = gripper_frame_id\n(no base transform)"]

  B -- Yes --> D["T_base_camera from TF2\nrobot_base_frame_id ← gripper_frame_id\n4×4 homogeneous matrix"]

  D --> E["SE(3) multiplication\nT_grasp_base = T_base_camera × T_grasp_cam"]

  E --> F["Apply constant offset\np_final = p_grasp_base + grasp_offset_base_xyz\nabsorbs hand-eye calibration residual\ndefault: [0.0, 0.0, 0.0] m"]

  F --> F2["Axis remap: R @ GRASP_TO_TCP_AXES\ngrasp (X=closing, Z=approach)\n→ LIO TCP (X=approach, Y=closing)"]

  F2 --> G["Rotation matrix → quaternion\nShepperd's method\nconstraint: w ≥ 0 (canonical hemisphere)"]

  G --> H["PoseStamped\nframe_id = LIO_base_link\nposition: x, y, z (m)\norientation: qx, qy, qz, qw"]

  H --> I["Publish ~/best_grasp\nPublish /commanded_pose"]
```

### Coordinate conventions

| Symbol | Meaning |
|--------|---------|
| `T_base_camera` | 4×4 rigid transform: points in camera frame → robot base frame |
| `T_grasp_cam` | 4×4 pose of gripper in camera frame (from server) |
| `T_grasp_base` | 4×4 pose of gripper in robot base frame (after transform) |
| `grasp_offset_base_xyz` | Constant position correction in base frame (metres) |
| Quaternion order | `(x, y, z, w)` in all ROS messages |
| Pinocchio internal | `(w, x, y, z)` — conversion applied before/after IK calls |

---

## 4. TF Frame Tree

Complete tree as observed at runtime (`ros2 run tf2_tools view_frames`):

```mermaid
graph TD
  MAP["map"]
  ODOM["odom"]
  BF["base_footprint\n(mobile platform, dynamic ~40 Hz)"]
  PBL["platform_base_link\n(static)"]
  LBL["LIO_base_link\n(arm assembly mount, static)\n= robot_base_frame_id\n= Pinocchio IK world frame"]
  LRBL["LIO_robot_base_link\n(arm mount link in URDF)\nstatic: t=[0,0,0.266] R=Rz(90°)"]
  L12["lio_link12"]
  DOTS["... lio_link23→34→45→56→lio_link6G\n(all dynamic ~10 Hz)"]
  GIF["lio_gripper_interface_link\n(static)"]
  CL["camera_link\n(static)"]
  CCF["camera_color_frame\n(static)"]
  OPT["camera_color_optical_frame\n(gripper_frame_id, static)"]
  GL["lio_gripper_link\n(static)"]
  TCP["lio_tcp_link\n(ik_tip_link, static)"]
  GB["grasp_best\n(published by node ~10 Hz)"]
  GBC["grasp_best_cam\n(debug, ~10 Hz)"]

  MAP --> ODOM --> BF --> PBL --> LBL
  LBL -->|"static\nt=[0,0,0.266] R=Rz(90°)"| LRBL
  LRBL -->|"dynamic"| L12 --> DOTS --> GIF
  GIF --> CL --> CCF --> OPT
  GIF --> GL --> TCP
  LRBL --> GB
  OPT --> GBC
```

### Frame roles

| Frame | Role | Used by |
|---|---|---|
| `LIO_base_link` | Arm assembly mounting point on platform; root link of `lio_arm_reframed.urdf`; Pinocchio world frame | Output frame for `/commanded_pose` (`robot_base_frame_id`); IK TF lookup target (`ik_base_link`) |
| `LIO_robot_base_link` | Parent of `lio_link12` in full robot TF tree; **not present in the IK URDF**; 266 mm above + 90° from `LIO_base_link` | TF chain intermediate only (no longer used as an output frame) |
| `camera_color_optical_frame` | RealSense color sensor optical frame; all grasp poses are stored in this frame | TF lookup source; `gripper_frame_id` |

The two base frames are **not interchangeable**: they differ by a static transform of `t = [0, 0, 0.266] m`, `R = Rz(90°)`. All published commands and the IK check now use `LIO_base_link` (`robot_base_frame_id` = `ik_base_link` = `LIO_base_link`); `LIO_robot_base_link` only appears as an intermediate link inside the full-robot TF tree.

The static transform from `lio_gripper_interface_link` → `camera_link` is published by the production launch file ([grasp_pose_client.launch.py](../launch/grasp_pose_client.launch.py)) via a `StaticTransformBroadcaster`. Its rotation places the RealSense optical axis pointing forward along the robot's approach direction.

---

## 5. IK Feasibility Check

Enabled by setting the `ik_urdf_path` parameter. Uses Pinocchio's Gauss-Newton solver in two stages to avoid local minima.

```mermaid
flowchart TD
  A["Top-K grasp candidates\nin camera frame (from server)"] --> A2["TF lookup\nT = TF(LIO_base_link ← camera_optical)\ntransform to base frame in memory only"]
  A2 --> B["Collect joint seed q₀\nfrom /joint_states topic\nfallback: neutral pose"]

  B --> C["Stage 1 — Position-only IK\nGauss-Newton, LOCAL_WORLD_ALIGNED\n3-DOF (translation only)\nmax_iter iterations, eps threshold"]

  C --> D{Position error\n< 1 mm?}
  D -- No --> E["infeasible — discard"]
  D -- Yes --> F["Stage 2 — Full 6-DOF IK\nfrom warm position seed q₁\nmax_iter iterations, eps threshold"]

  F --> G{Full pose error\n< ik_eps?}
  G -- No --> E
  G -- Yes --> H["feasible"]

  E --> I["Collect passed[]\n(original camera-frame dicts,\nsoft scores intact)"]
  H --> I

  I --> J["POST /submit_ik_result\n{run_id, trace_id, grasps: passed[]}"]
  J --> K["Server stores IK-passing camera-frame grasps\nranking deferred to /select_and_execute"]
```

### URDF used for IK

The launch file configures `ik_urdf_path` to `lio_arm_reframed.urdf` (from the `panda_ik` package). This is a **trimmed arm-only URDF** whose Pinocchio frame list is:

```
universe → LIO_base_link → lio_joint1 → lio_link12 → ... → lio_tcp_link
```

`LIO_robot_base_link` is **not present** in this URDF. The Pinocchio world frame is `LIO_base_link`, so the TF lookup target `ik_base_link = "LIO_base_link"` is correct — the IK target SE3 expressed in `LIO_base_link` is interpreted directly as the Pinocchio world-frame target without any additional offset.

**Key parameters:**

| Parameter | Default | Deployed value | Role |
|-----------|---------|----------------|------|
| `ik_urdf_path` | `""` | `<panda_ik>/urdfs/lio_arm_reframed.urdf` | Path to robot URDF; empty disables IK |
| `ik_base_link` | `LIO_base_link` | `LIO_base_link` | Root link of IK URDF = Pinocchio world frame |
| `ik_tip_link` | `lio_tcp_link` | `lio_tcp_link` | End-effector link in URDF |
| `ik_max_iter` | `200` | `200` | Max Gauss-Newton iterations per stage |
| `ik_eps` | `1e-4` | `1e-4` | Convergence threshold (m / rad) |
| `ik_dt` | `0.1` | `0.1` | Newton step size |
| `ik_damp` | `1e-6` | `1e-6` | Damping factor for (J Jᵀ + λI)⁻¹ |

---

## 6. Web UI Poll Loop

A 0.5 s timer polls two server endpoints so the Web UI can trigger captures and publish grasp results without a direct ROS service call.

```mermaid
flowchart TD
  T["Poll Timer fires\nevery 0.5 s"] --> A["GET /poll_capture_request"]
  A --> B{Capture\nrequested?}

  B -- Yes --> C["_do_upload_capture()\nPOST /upload_capture\nwith current color + depth frame"]
  C --> T

  B -- No --> D["GET /poll_publish"]
  D --> E{Publish\nrequested?}

  E -- No --> T

  E -- Yes --> F{mode?}

  F -- ik_check --> G["_check_ik_feasibility(grasps_cam)\ninternal TF to LIO_base_link\nPinocchio IK per candidate"]
  G --> H["POST /submit_ik_result\n{run_id, trace_id, grasps: passed[]}\ncamera-frame dicts returned unchanged"]
  H --> T

  F -- execute --> I["_transform_to_base()\nTF(LIO_base_link ← camera_optical)\n+ grasp_offset_base_xyz\n+ grasp→TCP axis remap"]
  I --> J["_publish_visualisation()\n~/best_grasp, ~/grasps\n/commanded_pose"]
  J --> T
```

---

## 7. Image Encoding

Handled by [image_conversion.py](../grasp_pose_client/image_conversion.py).

```mermaid
flowchart LR
  subgraph Color
    C1["sensor_msgs/Image\nencoding: rgb8 / bgr8\n         rgba8 / bgra8"] --> C2["cv_bridge → ndarray"]
    C2 --> C3["cv2.imencode('.png')\n→ bytes"]
  end

  subgraph Depth
    D1["sensor_msgs/Image\nencoding: 16UC1 (mm)\n         32FC1 (m)"] --> D2["cv_bridge → ndarray"]
    D2 --> D3["16UC1: ÷ 1000 → float32 m\n32FC1: as-is float32 m"]
    D3 --> D4["np.save() → .npy bytes"]
  end

  subgraph Intrinsics
    K1["sensor_msgs/CameraInfo\n.k (9 floats, row-major)"] --> K2["reshape 3×3\n→ JSON string"]
  end
```

---

## 8. ROS 2 Callback Concurrency Model

```mermaid
graph TD
  subgraph ReentrantCallbackGroup
    S1["_on_synced_frames()\ncolor + depth + info callback\nUpdates _latest cache under lock"]
  end

  subgraph MutuallyExclusiveCallbackGroup A
    S2["_handle_request_grasp()\nService handler\nBlocks while HTTP request is in flight"]
  end

  subgraph MutuallyExclusiveCallbackGroup B
    S3["_broadcast_tf()\n10 Hz timer\nRe-publishes last known grasp TF"]
  end

  subgraph MutuallyExclusiveCallbackGroup C
    S4["_poll_publish()\n2 Hz timer\nPolls server for Web UI events"]
  end
```

The sync callback uses a `ReentrantCallbackGroup` so that new frames can arrive and update `_latest` even while an HTTP request is blocked inside `_handle_request_grasp`. The service handler uses a `MutuallyExclusiveCallbackGroup` to ensure only one in-flight request at a time, matching the server-side lock.

---

## 9. Key Parameters Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `server_url` | `http://localhost:8765` | Grasp server base URL |
| `color_topic` | `/camera/camera/color/image_raw` | Color image subscription |
| `depth_topic` | `/camera/camera/aligned_depth_to_color/image_raw` | Aligned depth subscription |
| `camera_info_topic` | `/camera/camera/color/camera_info` | Camera intrinsics |
| `sync_queue_size` | `10` | ApproximateTimeSynchronizer queue depth |
| `sync_slop_s` | `0.05` | Color/depth time sync tolerance (s) |
| `gripper_frame_id` | `camera_color_optical_frame` | TF source frame (grasps from server) |
| `robot_base_frame_id` | `LIO_base_link` | TF target frame (published poses) |
| `tf_timeout_s` | `0.2` | TF lookup timeout (s) |
| `grasp_offset_base_xyz` | `[0.0, 0.0, 0.0]` | Extrinsic bias correction in base frame (m) |
| `max_snapshot_age_s` | `2.0` | Reject frames older than this (s) |
| `request_timeout_s` | `60.0` | HTTP POST timeout (s) |
| `default_top_k` | `1` | Default number of grasp candidates to return |
| `default_num_candidates` | `1` | Default VLM proposal count |
| `ik_urdf_path` | `""` | Path to URDF for IK; empty = IK disabled |
| `ik_base_link` | `LIO_base_link` | URDF root link for IK |
| `ik_tip_link` | `lio_tcp_link` | URDF end-effector link for IK |
| `joint_states_topic` | `/joint_states` | Joint state topic for IK seed |
| `probe_health_on_startup` | `true` | Call `/health` on server at node startup |
