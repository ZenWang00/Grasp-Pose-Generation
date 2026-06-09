# `/commanded_pose` 消费链路

本文描述 `grasp_pose_client` 向 `/commanded_pose` 发布 `PoseStamped` 之后，下游各节点如何消费该消息、完成关节角求解，并最终驱动机械臂运动。

> 上游发布逻辑见 [coordinate_transform_chain.md](coordinate_transform_chain.md)。

---

## 总体链路

```
/commanded_pose  (PoseStamped, frame_id=LIO_base_link)
        │
        ▼
panda_ik_teleop 节点  [main_teleop.cpp]
  ├─ TF2 变换: frame_id → LIO_base_link（frame_id 已是 LIO_base_link，为恒等变换）
  └─ PANOC IK 求解 (Rust, k::Chain)
        │
        ▼
/panda_ik_teleop/output  (Float64MultiArray, 6×关节角, 弧度)
        │
        ▼
ik_stream_to_action 节点  [ik_stream_to_action.py]
  ├─ 弧度 → 角度
  ├─ 速率限制 (≤2°/tick, 50 Hz)
  └─ 发送 ExecuteFunction action goal
        │
        ▼
/execute_function  (fp_core_msgs/action/ExecuteFunction)
  action = "move_joints"
  joint_position = [deg×6]
        │
        ▼
机械臂硬件
```

---

## 节点 1：`panda_ik_teleop`

**源文件**：[cpp_src/main_teleop.cpp](../../panda-ik/cpp_src/main_teleop.cpp)  
**节点名**：`panda_ik_teleop`  
**运行频率**：100 Hz 主循环

### 1.1 订阅 `/commanded_pose`

```cpp
commanded_pose_sub = node->create_subscription<PoseStamped>(
    "/commanded_pose", 1,
    [&](PoseStamped::SharedPtr msg) {
        // TF2 帧变换：msg->header.frame_id 现为 "LIO_base_link"，
        // 与目标帧相同，transform() 为恒等变换（不改变数值）。
        PoseStamped transformed =
            tfBuffer.transform(*msg, "LIO_base_link", tf2::durationFromSec(0.2));
        commandedPose.pose = transformed.pose;

        commandedVel = Twist();   // 清零速度（静止目标）
        frame_id = "lio_tcp_joint";
        start = true;             // 触发主循环
    });
```

**说明**：`/commanded_pose` 现在发布在 `LIO_base_link` 系（`robot_base_frame_id` 已统一到
IK URDF 根帧），`panda_ik_teleop` 的 `tfBuffer.transform(..., "LIO_base_link")` 为恒等变换，
数值不变，代码无需改动。TF 失败时降级：直接用原始 pose。

### 1.2 主循环（100 Hz）

每次循环对当前 `commandedPose` 执行：

```
1. 速度积分（teleop 模式）：
   position += vel * (1/freq)
   orientation = motion_rot * orientation  → 归一化

2. 调用 Rust IK 求解器：
   solve(joint_angles, "lio_tcp_joint",
         position, orientation, velocity, errors, w=1.0)
   → 更新 joint_angles[6]（弧度）

3. 连续性检查（initialized 后）：
   |joint_angles[i] - prev[i]| ≤ 0.1 rad  →  valid
   否则检查末端位置是否在 ±5 cm 范围内

4. 发布 /panda_ik_teleop/output（Float64MultiArray, 6×rad）
5. 发布 panda_commanded_pose（PoseStamped，用于 RViz 调试）
```

---

## 节点 1 核心：Rust PANOC IK 求解器

**源文件**：[src/lib.rs](../../panda-ik/src/lib.rs)  
**库**：`k`（前向运动学）+ `optimization_engine`（PANOC 优化器）

### IK 问题定义

```
minimize   J(q) = 100·||p(q) - p_target||²
                + w·||angle(R(q), R_target)||²
                + movement_cost(q, q0)

subject to lb ≤ q ≤ ub
```

| 项 | 含义 |
|----|------|
| `p(q)` | 当前关节角 `q` 下 `lio_tcp_joint` 的笛卡尔位置 |
| `p_target` | 目标位置（来自 TF 变换后的 pose） |
| `w·rotation_cost` | 姿态误差（角度平方），权重 `w=1.0` |
| `movement_cost` | 惩罚与初始状态 `q0` 的偏差，鼓励小幅运动 |

### 关节限位（弧度）

| 关节 | 下限 | 上限 |
|------|------|------|
| joint 1 | -2.793 | +2.793 |
| joint 2 | -1.745 | +1.745 |
| joint 3 | -1.745 | +1.745 |
| joint 4 | -2.793 | +2.793 |
| joint 5 | -1.745 | +1.745 |
| joint 6 | -2.793 | +2.793 |

（实际取值收缩 ±0.1 rad 作为软边界）

### 求解器参数

| 参数 | 值 | 含义 |
|------|----|------|
| 最大迭代次数 | 50 | PANOC 内层迭代 |
| 最大求解时间 | 7 ms | 超时则返回当前最优 |
| 梯度方法 | 有限差分 | h = 1000·ε_f64 |
| PANOC cache | 6 DOF, tol=1e-6 | 全局单例 |

### 错误标志（`errors[4]`）

| 索引 | 含义 |
|------|------|
| `[0]` | 优化器抛出异常 → fallback 到 `joint_angles` |
| `[1]` | 未收敛（时间超限或迭代超限） |
| `[2,3]` | 未使用 |

---

## 节点 2：`ik_stream_to_action`

**源文件**：[lio_specific_pkg_ros2/ik_stream_to_action.py](../../lio_specific_pkg_ros2/lio_specific_pkg_ros2/ik_stream_to_action.py)  
**节点名**：`joint_io_client`

### 订阅 `/panda_ik/output`（或指定 topic）

```python
def _ik_cb(self, msg: Float64MultiArray):
    arm6_deg = [math.degrees(x) for x in msg.data[:6]]
    self._latest_ik_deg = arm6_deg   # 仅缓存，不立即发送
```

### 控制循环（50 Hz 定时器）

```
1. 速率限制：
   desired[i] = clamp(target[i], last[i] ± max_step_deg)
   默认 max_step_deg = 2.0°/tick

2. 死区过滤：
   if |desired[i] - last_cmd[i]| < eps (0.02°): 跳过

3. 发送 ExecuteFunction action goal：
   action  = "move_joints"
   bridge  = "core"
   arguments = {
       "joints": [1,2,3,4,5,6],
       "joint_position": [deg×6]
   }
```

### 速率限制的作用

100 Hz IK 每次更新关节角最多 2°，50 Hz 控制循环确保发送频率可控，防止突变指令损伤电机。

---

## 完整数据流表

| 阶段 | 话题/接口 | 消息类型 | 帧/单位 |
|------|-----------|----------|---------|
| VLA Server → 客户端 | HTTP JSON | dict | `camera_color_optical_frame` |
| 客户端 → IK节点 | `/commanded_pose` | `PoseStamped` | `LIO_base_link` |
| IK节点内（TF 恒等） | `commandedPose` (内存) | `Pose` | `LIO_base_link` |
| IK节点 → 动作桥 | `/panda_ik_teleop/output` | `Float64MultiArray` | 弧度 × 6 |
| 动作桥 → 硬件 | `/execute_function` | `ExecuteFunction` goal | 角度(°) × 6 |

---

## 两种 IK 节点的区别

项目中存在两个 IK 节点实现：

| | `main_teleop.cpp`（当前使用） | `main.cpp`（旧版） |
|--|-------------------------------|-------------------|
| 节点名 | `panda_ik_teleop` | `panda_ik` |
| `/commanded_pose` 处理 | **有 TF 变换**（→ `LIO_base_link`） | 无变换，直接使用 |
| teleop 速度支持 | 有（`/input` twist） | 无 |
| 方向指令支持 | 有（`/directions` string） | 无 |
| 轨迹跟随 | 有（`/simulator/end_effector_pose`） | 无 |
| `weighted_pose` 模式 | 无 | 有（`/weighted_pose`） |

`main_teleop.cpp` 是当前实际运行的版本（见最近 git commit）。

---

## 关键注意事项

1. **坐标系统一**：`robot_base_frame_id = "LIO_base_link"`，与 Pinocchio 可行性检查（`ik_base_link`）和 Rust PANOC IK 的 URDF 根帧完全一致。`panda_ik_teleop` 的 TF 转换为恒等变换，不改变数值。

2. **IK 收敛时间**：单次求解限 7 ms（50 次迭代），未收敛时 `errors[1]=true` 但仍返回当前最优解，不会阻塞控制循环。

3. **速率限制**：`ik_stream_to_action` 的 2°/tick 限制是最后一道保护，防止 IK 大跳变直接下发到硬件。

4. **关节初始状态**：`joint_angles` 硬编码初始值 `{-1.621, -1.197, 1.274, 0.025, 1.482, -0.057}`（rad），每次 IK 成功后就地更新，作为下次求解的热启动种子。

5. **grasp_offset_base_xyz 轴方向**：offset 在 `LIO_base_link` 系下生效（+x=前伸, +y=左, +z=上），与之前文档记录的 `LIO_robot_base_link` 系（两者差 Rz(90°)）不同，重新标定时需注意。
