from std_msgs.msg import Float64MultiArray
import time

TARGET_TOPIC = "/grasp_target_myp"
JOINT_VEL = 5.0
JOINT_ACC = 15.0
ARM_CONFIG = "c4"
HEARTBEAT_INTERVAL_S = 10.0

myp_running = True
pending = None

def target_cb(msg):
    global pending
    pending = list(msg.data)

node = ros_handler.ros_node()
node.create_subscription(Float64MultiArray, TARGET_TOPIC, target_cb, 1)
print("grasp_executor_myp: listening on", TARGET_TOPIC)

last_heartbeat = time.monotonic()
pose_count = 0

while myp_running:
    now = time.monotonic()
    if now - last_heartbeat >= HEARTBEAT_INTERVAL_S:
        last_heartbeat = now
        print("grasp_executor_myp: alive, waiting on", TARGET_TOPIC, "- poses executed:", pose_count)

    if pending is not None:
        vals = pending
        pending = None

        if len(vals) < 6:
            print("grasp_executor_myp: bad message, expected 6 values, got", len(vals))
        else:
            x_mm = vals[0]
            y_mm = vals[1]
            z_mm = vals[2]
            roll = vals[3]
            pitch = vals[4]
            yaw = vals[5]

            print("grasp_executor_myp: move_pose target")
            print("  xyz (mm) :", round(x_mm, 1), round(y_mm, 1), round(z_mm, 1))
            print("  rpy (deg):", round(roll, 1), round(pitch, 1), round(yaw, 1))

            current = get_current_pose()
            coords = current.coordinates.from_components(x_mm, y_mm, z_mm, roll, pitch, yaw)
            pose = create_pose(coords, ARM_CONFIG)
            result = move_pose(pose, joint_velocity=JOINT_VEL, joint_acceleration=JOINT_ACC, block=True)
            pose_count = pose_count + 1
            print("grasp_executor_myp: move_pose result:", result)

    time.sleep(0.05)
