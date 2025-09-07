#!/usr/bin/env python3
"""
UR5e Control Tools - ROSA-compatible tools for robot control
Implements joint control, cartesian control, and state monitoring
"""
import rclpy
from langchain.agents import tool
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from custom_msgs.srv import MoveToPose, CustomSwitchController
from controller_manager_msgs.srv import SwitchController
import numpy as np
import threading
import time
import tf2_ros
import rclpy.duration

# Global node and state variables
_shared_node = None
_joint_states = None
_current_pose = None
_joint_pub = None
_cartesian_client = None
_controller_client = None
_initialized = False

def initialize_node():
    """Initialize ROS2 node and all publishers/subscribers/clients"""
    global _shared_node, _joint_pub, _cartesian_client, _controller_client, _initialized
    
    if _initialized:
        return
    
    print("🚀 Initializing UR5e control node...")
    rclpy.init()
    _shared_node = rclpy.create_node('ur5e_rosa_control')
    
    # Publishers
    _joint_pub = _shared_node.create_publisher(
        JointTrajectory,
        '/scaled_joint_trajectory_controller/joint_trajectory',
        10
    )
    
    # Service clients
    _cartesian_client = _shared_node.create_client(MoveToPose, "move_to_pose")
    _controller_client = _shared_node.create_client(
        CustomSwitchController, "controller_switcher"
    )
    
    # Subscribers
    _shared_node.create_subscription(
        JointState, '/joint_states', _joint_state_callback, 10
    )
    _shared_node.create_subscription(
        PoseStamped, '/cartesian_motion_controller/current_pose', _pose_callback, 10
    )
    
    # Spin in background
    def spin_node():
        try:
            rclpy.spin(_shared_node)
        except Exception as e:
            print(f"❌ Spin thread error: {e}")
    
    threading.Thread(target=spin_node, daemon=True).start()
    
    _initialized = True
    print("✅ UR5e control node ready")

def _joint_state_callback(msg):
    """Callback to update joint states"""
    global _joint_states
    _joint_states = msg

def _pose_callback(msg):
    """Callback to update current pose"""
    global _current_pose
    _current_pose = msg

def _wait_for_data(data_var, timeout=5.0, data_name="data"):
    """Helper to wait for ROS data with timeout"""
    start_time = time.time()
    while data_var is None and (time.time() - start_time) < timeout:
        time.sleep(0.1)
    if data_var is None:
        raise TimeoutError(f"Timeout waiting for {data_name}")

def init_ur5e_node():
    """Helper function to ensure node is initialized"""
    initialize_node()

@tool
def get_current_joint_states() -> str:
    """
    Get the current joint positions of the UR5e robot.
    Returns joint angles in both radians and degrees.
    """
    initialize_node()
    try:
        _wait_for_data(_joint_states, data_name="joint states")
        joint_names = ["shoulder_pan", "shoulder_lift", "elbow",
                       "wrist_1", "wrist_2", "wrist_3"]
        result = "🤖 Current Joint States:\n"
        for i, (name, angle) in enumerate(zip(joint_names, _joint_states.position)):
            degrees = np.rad2deg(angle)
            result += f"  {i+1}. {name}: {angle:.3f} rad ({degrees:.1f}°)\n"
        return result
    except TimeoutError:
        return "❌ Failed to get joint states - robot may not be connected"
    except Exception as e:
        return f"❌ Error getting joint states: {str(e)}"

@tool
def activate_controller(name: str) -> str:
    """
    Activate a specific robot controller directly via /controller_manager/switch_controller.
    Args:
        name: 'cartesian_motion_controller' or 'scaled_joint_trajectory_controller'
    """
    initialize_node()
    valid = ["cartesian_motion_controller", "scaled_joint_trajectory_controller"]
    if name not in valid:
        return f"❌ Invalid controller. Use: {valid}"

    # Create client for the controller_manager service
    cm_client = _shared_node.create_client(SwitchController, "/controller_manager/switch_controller")
    if not cm_client.wait_for_service(timeout_sec=5.0):
        return "❌ controller_manager service not available"

    req = SwitchController.Request()
    req.start_controllers = [name]
    req.stop_controllers = [c for c in valid if c != name]
    req.strictness = SwitchController.Request.BEST_EFFORT

    future = cm_client.call_async(req)
    rclpy.spin_until_future_complete(_shared_node, future, timeout_sec=10.0)
    if not future.done():
        return "❌ Controller switch timeout"
    resp = future.result()
    if resp.ok:
        typ = "Cartesian" if "cartesian" in name else "Joint"
        return f"✅ Activated {typ} controller"
    else:
        return f"❌ Failed to switch controllers: ok={resp.ok}"

@tool
def move_joint_angles(joint_angles: list, duration: float = 4.0) -> str:
    """
    Move UR5e joints to specified angles.
    Args:
      joint_angles: 6 angles in degrees
      duration: seconds
    """
    initialize_node()
    if len(joint_angles) != 6:
        return "❌ Error: Need exactly 6 joint angles"
    for a in joint_angles:
        if abs(a) > 360:
            return "❌ Joint angle exceeds ±360°"
    
    # Activate trajectory controller - call internal function, not .invoke()
    switch_result = _activate_controller_internal("scaled_joint_trajectory_controller")
    if "❌" in switch_result:
        return switch_result
    time.sleep(0.5)
    
    angles_rad = [np.deg2rad(a) for a in joint_angles]
    traj = JointTrajectory()
    traj.header.stamp = _shared_node.get_clock().now().to_msg()
    traj.joint_names = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    ]
    pt = JointTrajectoryPoint()
    pt.positions = angles_rad
    pt.time_from_start.sec = int(duration)
    pt.time_from_start.nanosec = int((duration % 1) * 1e9)
    traj.points.append(pt)
    _joint_pub.publish(traj)
    return f"✅ Moving joints to {joint_angles}° over {duration}s"


@tool()
def move_cartesian_relative(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> str:
    """
    Move the robot end-effector relative to its current position.
    Args:
        x, y, z in meters (max ±0.2m)
    """
    initialize_node()
    if abs(x) > 0.2 or abs(y) > 0.2 or abs(z) > 0.2:
        return "❌ Movement too large. Max ±0.2m"

    # Activate Cartesian controller
    switch_result = _activate_controller_internal("cartesian_motion_controller")
    if "❌" in switch_result:
        return switch_result
    time.sleep(0.5)

    try:
        # Lookup current pose via TF
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, _shared_node)
        trans = tf_buffer.lookup_transform(
            "base_link", "tool0", rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=2.0)
        )

        # Build a Pose from the transform and apply the delta
        target_pose = Pose()
        target_pose.position.x = trans.transform.translation.x + x
        target_pose.position.y = trans.transform.translation.y + y
        target_pose.position.z = trans.transform.translation.z + z
        target_pose.orientation = trans.transform.rotation

        if not _cartesian_client.wait_for_service(timeout_sec=5.0):
            return "❌ Cartesian service unavailable"

        req = MoveToPose.Request()
        req.target_pose = target_pose  # assign the Pose here

        future = _cartesian_client.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < 30:
            time.sleep(0.1)
        if not future.done():
            return "❌ Cartesian motion service timeout"

        resp = future.result()
        if resp.success:
            return f"✅ Moved ΔX={x:.3f},ΔY={y:.3f},ΔZ={z:.3f}m"
        else:
            return f"❌ Cartesian motion failed: {resp.message}"

    except Exception as e:
        return f"❌ Error in cartesian movement: {e}"


@tool()
def get_current_pose() -> str:
    """
    Get the current position and orientation of the robot end-effector.
    Returns: formatted string of X, Y, Z in meters.
    """
    initialize_node()
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, _shared_node)
    try:
        trans = tf_buffer.lookup_transform("base_link", "tool0", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
        p = trans.transform.translation
        return f"📍 Pose: X={p.x:.3f}m Y={p.y:.3f}m Z={p.z:.3f}m"
    except Exception as e:
        return f"❌ Could not fetch pose via TF: {e}"


@tool
def move_to_home_position() -> str:
    """Move UR5e to home joint configuration [0, -90, 90, -90, -90, 0]."""
    try:
        initialize_node()
        home_angles = [0, -90, 90, -90, -90, 0]
        
        # Call the internal function directly, not via .invoke()
        result = _move_joint_angles_internal(home_angles, 5.0)
        return result
        
    except Exception as e:
        return f"❌ Error moving to home position: {e}"

@tool
def emergency_stop() -> str:
    """Emergency stop - stop all robot motion immediately."""
    initialize_node()
    try:
        traj = JointTrajectory()
        traj.header.stamp = _shared_node.get_clock().now().to_msg()
        traj.joint_names = []
        _joint_pub.publish(traj)
        return "🛑 EMERGENCY STOP issued"
    except Exception as e:
        return f"❌ Emergency stop failed: {str(e)}"

# Internal helper functions (not tools)
def _activate_controller_internal(name: str) -> str:
    """Internal controller activation function"""
    initialize_node()
    valid = ["cartesian_motion_controller", "scaled_joint_trajectory_controller"]
    if name not in valid:
        return f"❌ Invalid controller. Use: {valid}"
    
    cm_client = _shared_node.create_client(SwitchController, "/controller_manager/switch_controller")
    if not cm_client.wait_for_service(timeout_sec=5.0):
        return "❌ controller_manager service not available"
    
    req = SwitchController.Request()
    # Use the new API fields
    req.activate_controllers = [name]  # Changed from start_controllers
    req.deactivate_controllers = [c for c in valid if c != name]  # Changed from stop_controllers
    req.strictness = SwitchController.Request.BEST_EFFORT
    req.timeout.sec = 10  # Set explicit timeout instead of 0
    
    future = cm_client.call_async(req)
    rclpy.spin_until_future_complete(_shared_node, future, timeout_sec=10.0)
    if not future.done():
        return "❌ Controller switch timeout"
    
    resp = future.result()
    if resp.ok:
        typ = "Cartesian" if "cartesian" in name else "Joint"
        return f"✅ Activated {typ} controller"
    else:
        return f"❌ Failed to switch controllers: ok={resp.ok}"


def _move_joint_angles_internal(joint_angles: list, duration: float = 4.0) -> str:
    """Internal joint movement function"""
    initialize_node()
    if len(joint_angles) != 6:
        return "❌ Error: Need exactly 6 joint angles"
    for a in joint_angles:
        if abs(a) > 360:
            return "❌ Joint angle exceeds ±360°"
    
    switch_result = _activate_controller_internal("scaled_joint_trajectory_controller")
    if "❌" in switch_result:
        return switch_result
    time.sleep(0.5)
    
    angles_rad = [np.deg2rad(a) for a in joint_angles]
    traj = JointTrajectory()
    traj.header.stamp = _shared_node.get_clock().now().to_msg()
    traj.joint_names = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    ]
    pt = JointTrajectoryPoint()
    pt.positions = angles_rad
    pt.time_from_start.sec = int(duration)
    pt.time_from_start.nanosec = int((duration % 1) * 1e9)
    traj.points.append(pt)
    _joint_pub.publish(traj)
    return f"✅ Moving joints to {joint_angles}° over {duration}s"
