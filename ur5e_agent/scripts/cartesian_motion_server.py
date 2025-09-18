#!/usr/bin/env python3
"""
Cartesian Motion Server - Handles pose-based robot movements
Uses cartesian_motion_controller to move end-effector to target poses
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_msgs.srv import MoveToPose
import time
from controller_manager_msgs.srv import ListControllers, LoadController, ConfigureController

class CartesianMotionServer(Node):
    def __init__(self):
        super().__init__("cartesian_motion_server")
        
        # Service to handle pose movement requests
        self.move_to_pose_service = self.create_service(
            MoveToPose, "move_to_pose", self.handle_move_to_pose
        )
        
        # Publisher to send target poses to cartesian controller
        self.target_pose_publisher = self.create_publisher(
            PoseStamped, "/cartesian_motion_controller/target_frame", 10
        )
        
        # Subscriber to get current robot pose
        self.current_pose_subscription = self.create_subscription(
            PoseStamped, "/cartesian_motion_controller/current_pose",
            self.pose_callback, 10
        )
        
        # Robot state variables
        self.current_pose = None
        self.position_tolerance = 0.05  # 1cm
        self.orientation_tolerance = 0.01
        self.timeout = 15.0  # seconds
        
        self.get_logger().info("🤖 Cartesian Motion Server initialized")

    def pose_callback(self, msg):
        """Update current robot pose"""
        self.current_pose = msg

    def handle_move_to_pose(self, request, response):
        """Handle incoming pose movement requests"""
        target_pose = request.target_pose
        self.get_logger().info(f"📍 Moving to pose: x={target_pose.position.x:.3f}, "
                              f"y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")
        
        # Create stamped pose message
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id = "base_link"
        target_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        target_pose_stamped.pose = target_pose
        
        # Publish target pose
        self.target_pose_publisher.publish(target_pose_stamped)
        
        # Wait for robot to reach target
        if self.wait_until_target_reached(target_pose_stamped):
            response.success = True
            response.message = "✅ Target pose reached successfully"
        else:
            response.success = False
            response.message = "❌ Failed to reach target pose within timeout"
        
        return response

    def wait_until_target_reached(self, target_pose):
        """Wait until robot reaches the target pose within tolerance"""
        start_time = self.get_clock().now().nanoseconds / 1e9
        
        while (self.get_clock().now().nanoseconds / 1e9 - start_time) < self.timeout:
            if self.current_pose is None:
                time.sleep(0.1)
                continue
            
            # Calculate position and orientation errors
            pos_error = [
                abs(self.current_pose.pose.position.x - target_pose.pose.position.x),
                abs(self.current_pose.pose.position.y - target_pose.pose.position.y),
                abs(self.current_pose.pose.position.z - target_pose.pose.position.z)
            ]
            
            # Check if within tolerance
            if all(err <= self.position_tolerance for err in pos_error):
                self.get_logger().info("🎯 Target pose reached!")
                return True
            
            time.sleep(0.1)
        
        self.get_logger().warn("⏰ Timeout - target pose not reached")
        return False

def main(args=None):
    rclpy.init(args=args)
    server = CartesianMotionServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()