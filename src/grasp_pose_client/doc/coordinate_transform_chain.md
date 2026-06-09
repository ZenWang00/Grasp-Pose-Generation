# 坐标变换链路：VLA Server → `/commanded_pose`

本文聚焦描述：VLA 服务器返回最佳抓取姿态之后，`grasp_pose_client_node` 对坐标进行的全部变换步骤，直到把 `PoseStamped` 发布到 `/commanded_pose` topic。

---

## 总体流程

```
VLA Server (HTTP)
  └─ 返回抓取姿态（相机光学坐标系）
       │
       ▼
Step 1  解析 JSON → 构造 T_grasp_cam（4×4，SE3）
       │
       ▼
Step 2  TF2 查询：LIO_robot_base_link ← camera_color_optical_frame
        → 得到 T_base_camera（4×4）
       │
       ▼
Step 3  矩阵乘法：T_grasp_base = T_base_camera × T_grasp_cam
       │
       ▼
Step 4  施加手眼标定残差修正：p_final = p_grasp_base + grasp_offset_base_xyz
       │
       ▼
Step 5  旋转矩阵 → 四元数（Shepperd 法）
       │
       ▼
Step 6  打包为 geometry_msgs/PoseStamped
        frame_id = "LIO_robot_base_link"
       │
       ├─ publish → ~/best_grasp      (latched，供 RViz)
       ├─ publish → ~/grasps          (latched，所有 Top-K)
       └─ publish → /commanded_pose   (下游执行消费)
```

---

## 详细步骤说明

### Step 1 — 解析服务器响应，构造 T_grasp_cam

代码位置：[`_transform_to_base()`](../grasp_pose_client/grasp_pose_client_node.py#L582)

服务器每个 grasp entry（dict）有两种格式：

| 字段 | 含义 |
|------|------|
| `pose_4x4` | 4×4 齐次变换矩阵，直接使用 |
| `position_xyz` + `quaternion_xyzw` | 位置向量 + 四元数，客户端自行构造旋转矩阵再拼成 4×4 |

四元数 → 旋转矩阵（当 `pose_4x4` 不存在时）：

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

所有坐标系均为 **`camera_color_optical_frame`**（RealSense 彩色相机光学坐标系，Z 轴指向场景）。

---

### Step 2 — TF2 查询 T_base_camera

代码位置：[`_lookup_T_base_camera()`](../grasp_pose_client/grasp_pose_client_node.py#L549)

```
lookup_transform(
    target_frame = "LIO_base_link",              # robot_base_frame_id（IK URDF 根帧）
    source_frame = "camera_color_optical_frame", # gripper_frame_id
    stamp = color_msg.header.stamp,              # 图像时间戳（服务调用路径）
              或 Time()                          # 最新可用（轮询路径）
)
```

TF 链路（运行时完整路径）：

```
camera_color_optical_frame
  ← camera_color_frame          (RealSense driver，静态)
    ← camera_link               (RealSense driver，静态)
      ← lio_gripper_interface_link  (手眼标定静态 TF，launch 文件发布)
        ← lio_link6G / lio_link56 / ... / lio_link12
          ← LIO_robot_base_link ← LIO_base_link ← ... ← map
```

`lio_gripper_interface_link` → `camera_link` 这段静态 TF 由 launch 文件的
`static_transform_publisher` 节点发布（[grasp_pose_client.launch.py](../launch/grasp_pose_client.launch.py#L115)）：

```
手眼标定结果：
  平移：x=-0.10 m, y=0.0, z=+0.052 m
  旋转：RPY(roll=0, pitch=-π/2, yaw=0)  →  Ry(-90°)
  四元数：qx=0, qy=-0.7071, qz=0, qw=0.7071
```

**查询失败时的降级处理**：如果 TF 超时（默认 0.2 s），`T_base_camera` 返回 `None`，
姿态直接用相机坐标系发布（`frame_id = gripper_frame_id`），下游收到的是未变换坐标。

---

### Step 3 — SE(3) 矩阵乘法

代码位置：[`_transform_to_base()` L611](../grasp_pose_client/grasp_pose_client_node.py#L611)

```python
T_grasp_base = T_base_camera @ T_grasp_cam   # (4×4) @ (4×4)
R_base = T_grasp_base[:3, :3]
t_base = T_grasp_base[:3, 3]
```

这是标准刚体变换复合：把「抓取姿态相对于相机」直接变换到「抓取姿态相对于机器人底座」。

---

### Step 4 — 手眼标定残差修正

代码位置：[`_transform_to_base()` L614](../grasp_pose_client/grasp_pose_client_node.py#L614)

```python
if offset_base is not None:
    t_base = t_base + offset_base   # offset_base = grasp_offset_base_xyz
```

参数 `grasp_offset_base_xyz`（默认 `[0.0, 0.0, 0.0]`）用于吸收系统性外参偏差。
坐标轴约定（`LIO_base_link` 坐标系，即 IK URDF 根帧）：

| 轴 | 方向 |
|----|------|
| +x | 机械臂前伸方向 |
| +y | 左 |
| +z | 向上 |

> **注意**：`robot_base_frame_id` 从旧的 `LIO_robot_base_link` 改为 `LIO_base_link` 后，
> 轴定义随之变化（两帧差 Rz(90°)）。任何基于旧轴方向的历史 offset 标定值均需重新校准。

---

### Step 5 — 旋转矩阵 → 四元数

代码位置：[`_transform_to_base()` L617–L649](../grasp_pose_client/grasp_pose_client_node.py#L617)

使用 **Shepperd 方法**（数值稳定的四分支算法，避免接近 0 的除法）：

```
trace > 0   → 标准公式
R[0,0] 最大 → 以 qx 为主元
R[1,1] 最大 → 以 qy 为主元
R[2,2] 最大 → 以 qz 为主元
```

规范化与半球约束：
```python
q = q / |q|          # L2 归一化
if q[3] < 0: q = -q  # 强制 w ≥ 0（唯一表示）
```

输出格式：`[qx, qy, qz, qw]`（ROS 标准）

---

### Step 6 — 打包 PoseStamped 并发布

代码位置：[`_build_pose_stamped()`](../grasp_pose_client/grasp_pose_client_node.py#L923)，
[`_publish_visualisation()`](../grasp_pose_client/grasp_pose_client_node.py#L688)

```python
msg = PoseStamped()
msg.header.stamp    = stamp        # 图像时间戳（保留时序）
msg.header.frame_id = "LIO_base_link"   # robot_base_frame_id = IK URDF 根帧
msg.pose.position.{x,y,z}         = t_base
msg.pose.orientation.{x,y,z,w}    = q
```

发布逻辑（`_publish_visualisation`）：

```python
best = response.grasps[0]          # Top-1 最高分抓取
self._best_pose_pub.publish(best)  # ~/best_grasp  (latched)
self._commanded_pose_pub.publish(best)  # /commanded_pose
# 全部 Top-K 也发布到 ~/grasps (PoseArray, latched)
```

**只有 `grasps[0]`（最高分）被发到 `/commanded_pose`。**

---

## 两条触发路径的差异

| | 服务调用路径 | Web UI 轮询路径 |
|---|---|---|
| 触发方 | ROS2 服务调用 `~/request_grasp` | 0.5 Hz 定时器轮询 `/poll_publish` |
| TF 时间戳 | `color_msg.header.stamp` | `Time()`（最新可用） |
| IK 检查 | 可选（`ik_bypass=True` 默认跳过） | 同上 |
| 发布路径 | 相同：`_transform_to_base` → `_publish_visualisation` | 相同 |

---

## 调试辅助输出

除 `/commanded_pose` 外，节点还发布以下调试 topic，可在 RViz 中叠加对比：

| Topic | 内容 |
|-------|------|
| `~/best_grasp_camera` | 服务器原始相机坐标系姿态（未变换） |
| `~/grasps_camera` | 全部 Top-K 原始相机坐标系姿态 |
| TF frame `grasp_best` | 以 `LIO_robot_base_link` 为父帧的 TF，10 Hz 刷新 |
| TF frame `grasp_best_cam` | 以 `camera_color_optical_frame` 为父帧的 TF，10 Hz 刷新 |

若 RViz 中两套 TF 帧叠合，说明 TF 变换正确；若有位移，说明外参标定有残差，可调 `grasp_offset_base_xyz`。
