from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
import math
import time


JOINT_IDS = [1, 2, 3, 4, 5, 6]
VELOCITY  = 25.0
ACCEL     = 90.0
BLOCK     = False

latest = [0.0] * 6
got_msg = False

# Thresholds for detecting open/close (in degrees)
GRIPPER_OPEN_THRESHOLD = 20.0    # adjust if need
GRIPPER_CLOSE_THRESHOLD = 5.0    # near fully closed

gripper_open = False  # current boolean state

myp_running = True


# callback to stop immediately when a message is received on /myp_state
def app_state_cb(msg: String):
    global myp_running
    print("[move_joint] Stop requested via /myp_commanded_state")

    if msg.data == "stop":
        myp_running = False


def joint_states_cb(msg: JointState):
    global latest, got_msg, gripper_open

    if not msg.position:
        return

    vals = list(msg.position)

    print('values', vals)
    if len(vals) < 6:
        vals += [0.0] * (6 - len(vals))
    latest = vals[:6]

    gripper_angle_deg = vals[-1]

    if not got_msg:
        print("First JointState received:", latest)
        got_msg = True

    # Detect open / close using degree thresholds
    if (gripper_angle_deg > GRIPPER_OPEN_THRESHOLD) and not gripper_open:
        print("Gripper opened")
        gripper_open = True
        tool_pick()

    elif (gripper_angle_deg < GRIPPER_OPEN_THRESHOLD) and gripper_open:
        print("Gripper closed")
        gripper_open = False
        tool_place()

    # Always move the robot joints
    move_joints(JOINT_IDS, latest,
                joint_velocity=VELOCITY,
                joint_acceleration=ACCEL,
                relative=False,
                block=BLOCK)

    # print("ready: move_joints called")
    print(f"gripper_angle: {gripper_angle_deg:.2f}°, gripper_open: {gripper_open}")




node = ros_handler.ros_node() 
node.create_subscription(JointState, "/ik_interface/joint_states_lio", joint_states_cb, 10) 
node.create_subscription(String, "/myp_commanded_state", app_state_cb, 10) 

print("ready: subscribed to /joint_states_lio") 


while True:
    if not myp_running:
        print("[move_joint] Exiting as requested.")
        stop_application()
    time.sleep(0.1)   
