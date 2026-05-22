# grasp_pose_client

ROS2 client node that bridges a RealSense RGBD stream to the remote VLA grasp HTTP
server hosted by the `vla-grasp-server` repo.

## Data flow

```
realsense2_camera ──> /camera/camera/color/image_raw            ┐
                       /camera/camera/aligned_depth_to_color/    ├─► grasp_pose_client ─HTTP─► vla-grasp-server
                       image_raw                                 │           │
                       /camera/camera/color/camera_info          ┘           ▼
                                                                       PoseArray + PoseStamped
                                                                       (camera frame)
```

- The node subscribes to the standard `realsense2_camera` triple and runs an
  `ApproximateTimeSynchronizer` to cache the latest synced snapshot.
- On each `RequestGrasp` service call it converts that snapshot to the multipart
  payload the grasp server expects (RGB PNG, depth `.npy` in meters, intrinsics
  as JSON), POSTs it, and turns the response into `geometry_msgs/PoseStamped[]`.
- The top grasp is also published to `~/best_grasp`, and the full sorted list to
  `~/grasps` (PoseArray) for RViz.

## Dependencies

Beyond what is already installed on the system:

```bash
sudo apt install ros-jazzy-realsense2-camera ros-jazzy-cv-bridge
# python-side
pip install --user requests  # or: sudo apt install python3-requests
```

This package also depends on the sibling `grasp_pose_client_msgs` package (built
in the same workspace).

## Build

```bash
cd ~/Grasp_Pose_Generation
colcon build --packages-select grasp_pose_client_msgs grasp_pose_client
source install/setup.bash
```

## Run

Three shells:

### 1. Start the RealSense driver

```bash
ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true \
    pointcloud.enable:=false
```

(Adjust the realsense2_camera launch file name to whatever your installed version
uses; `rs_launch.py` is the canonical one.)

### 2. Start the grasp HTTP server

In the `vla-grasp-server` repo:

```bash
export GEMINI_API_KEY=YOUR_KEY
./scripts/run_server.sh
```

### 3. Start the client node

```bash
ros2 launch grasp_pose_client grasp_pose_client.launch.py \
    server_url:=http://localhost:8765
```

If the server lives on a different machine, point `server_url` at it
(`http://<server-host>:8765`).

## Triggering a grasp

```bash
ros2 service call /grasp_pose_client/request_grasp \
    grasp_pose_client_msgs/srv/RequestGrasp \
    "{task_spec: 'Target: the blue bottle', top_k: 1}"
```

The response carries:

- `grasps[]` — `geometry_msgs/PoseStamped[]` in the camera frame
  (header.frame_id is copied from the source `Image.header.frame_id`)
- `scores[]` — Contact-GraspNet scores aligned with `grasps[]`
- `widths[]` — gripper widths in meters (`NaN` if the server didn't derive one)
- `run_id` — server-side run identifier; useful when debugging via the server's
  `output_vg/api_<run_id>/` directory

## RViz tips

Add the following displays in RViz (Fixed Frame = your robot's camera frame,
e.g. `camera_color_optical_frame`):

- `PoseStamped` on topic `/grasp_pose_client/best_grasp` — single best grasp
- `PoseArray` on topic `/grasp_pose_client/grasps` — top-K array
- `Image` on `/camera/camera/color/image_raw` — sanity check that the source
  frame matches what the VLM saw

The PoseStamped's x-axis points along the gripper's base (closing direction), and
its z-axis is the approach direction Contact-GraspNet emits.

## Parameters (override via launch args or `ros2 param set`)

| name | default | meaning |
| --- | --- | --- |
| `server_url` | `http://localhost:8765` | Base URL of the grasp HTTP server |
| `color_topic` | `/camera/camera/color/image_raw` | Color image subscription |
| `depth_topic` | `/camera/camera/aligned_depth_to_color/image_raw` | Aligned-depth subscription |
| `camera_info_topic` | `/camera/camera/color/camera_info` | Intrinsics subscription |
| `sync_slop_s` | `0.05` | ApproximateTimeSynchronizer slop |
| `request_timeout_s` | `60.0` | HTTP timeout |
| `default_top_k` | `1` | Default top-K (overridable per request) |
| `default_num_candidates` | `1` | Default VLM proposals |
| `max_snapshot_age_s` | `2.0` | Reject calls if the last synced frame is older |
| `probe_health_on_startup` | `true` | Call `GET /health` at startup |

## Topic/service summary

| kind | name | type |
| --- | --- | --- |
| service | `~/request_grasp` | `grasp_pose_client_msgs/srv/RequestGrasp` |
| topic out | `~/best_grasp` | `geometry_msgs/PoseStamped` |
| topic out | `~/grasps` | `geometry_msgs/PoseArray` |
| topic in | `<color_topic>` | `sensor_msgs/Image` |
| topic in | `<depth_topic>` | `sensor_msgs/Image` |
| topic in | `<camera_info_topic>` | `sensor_msgs/CameraInfo` |

## Testing without a real RealSense

This package ships a small ROS2 node, `capture_replay_node`, that re-publishes a
saved `captures/<timestamp>/` directory (`camera_data.npy` + `color_preview.jpg`)
on the same topic names that `realsense2_camera` would use. With it you can run
the full ROS2 -> HTTP -> ROS2 path on existing data, no camera required.

### Stage 1 (HTTP only, no ROS2 at all)

Run the server's `scripts/smoke_test_grasp.py` first to confirm the server is
healthy:

```bash
cd ~/vla-grasp-server
.venv/bin/python scripts/smoke_test_grasp.py \
    --capture-dir captures/20260417_120019 \
    --task-spec "Target: the blue bottle"
```

Expected: `health: status=ok, worker_ready=True` followed by `top grasps: #1 score=... pos=[...] quat_xyzw=[...]`.
If this fails, ROS2 is not the problem; fix the server first.

### Stage 2 (ROS2 client -> server, replayed data)

Build, source, then in two shells:

```bash
# Shell A: vla-grasp-server (in its own .venv)
cd ~/vla-grasp-server && ./scripts/run_server.sh

# Shell B: replay + client (after `conda deactivate` + `source /opt/ros/jazzy/setup.bash`
#           + `source ~/Grasp_Pose_Generation/install/setup.bash`)
ros2 launch grasp_pose_client grasp_pose_client_with_replay.launch.py \
    capture_dir:=$HOME/vla-grasp-server/captures/20260417_120019 \
    server_url:=http://localhost:8765
```

The launch file spawns both `capture_replay_node` (publishing the saved frame at
30 Hz) and `grasp_pose_client_node` (subscribing + offering the service).
Trigger one inference:

```bash
# Shell C
ros2 service call /grasp_pose_client/request_grasp \
    grasp_pose_client_msgs/srv/RequestGrasp \
    "{task_spec: 'Target: the blue bottle', top_k: 1}"
```

Watch:

- service response carries `success: true` and a populated `grasps[]`
- `ros2 topic echo /grasp_pose_client/best_grasp` shows one `PoseStamped`
- RViz with PoseStamped on `/grasp_pose_client/best_grasp` shows an arrow
- the server's `output_vg/api_<run_id>/report.html` opens in a browser

If Stage 2 passes, the full client code path (subscriber sync, cv_bridge, HTTP,
PoseStamped publishing) is correct. The only remaining unknown is the real
RealSense's actual topic timing / intrinsics, which is what Stage 3 covers.

### Stage 3 (real RealSense)

Replace the replay node with the real driver:

```bash
ros2 launch realsense2_camera rs_launch.py \
    align_depth.enable:=true pointcloud.enable:=false
ros2 launch grasp_pose_client grasp_pose_client.launch.py \
    server_url:=http://localhost:8765
```

Trigger with the same `ros2 service call` as above. If Stage 2 worked but
Stage 3 fails, the issue is either topic naming (`color_topic` / `depth_topic`
parameters) or sync slop (`sync_slop_s`).

## Troubleshooting

### `ros2 topic list` shows nothing but the node is running

DDS discovery on this machine is slow and the default `RMW_IMPLEMENTATION` (Cyclone)
may drop multi-process discovery entirely. Symptoms: `ros2 topic list` only shows
`/parameter_events` and `/rosout`, but the publisher node's logs look healthy.

Workarounds:

```bash
# Either force FastRTPS (publishes work, discovery is slow but eventually finds topics):
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# And verify with a *continuous* command, not a single-shot one:
ros2 topic hz /camera/camera/color/image_raw     # works
ros2 topic echo /camera/camera/color/camera_info # works (no --once)
ros2 topic list                                  # may need 2-3 retries
```

`grasp_pose_client_node` is a continuous subscriber, so it always eventually
discovers the publisher. If a single `ros2 service call` fails with "no
synchronized snapshot", wait a few seconds and retry — discovery is the only
delay.

## Known limitations

- Only RealSense-style synchronized topics are supported. If you ever wire this
  to a sim that publishes color + depth on differently-named topics, remap them
  via the launch arguments above.
- The depth encoding must be `16UC1` (millimeters, RealSense default for
  `aligned_depth_to_color`) or `32FC1` (already in meters). Any other encoding
  triggers an error before the HTTP round trip.
- The node holds **one** in-flight request at a time (service uses a mutually
  exclusive callback group). This matches the server-side `asyncio.Lock` so we
  don't queue work the server can't process in parallel.
