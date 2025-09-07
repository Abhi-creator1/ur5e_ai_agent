#!/usr/bin/env python3
"""
Controller Switcher - Manages switching between robot controllers
Handles conflicts between joint and cartesian controllers
"""
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from custom_msgs.srv import CustomSwitchController

class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__('controller_switcher')
        
        # Define mutually exclusive controllers
        self.available_controllers = [
            "cartesian_motion_controller",
            "scaled_joint_trajectory_controller"
        ]
        
        # Service to handle controller switching
        self.controller_switcher_service = self.create_service(
            CustomSwitchController, "controller_switcher", 
            self.handle_controller_switch
        )
        
        # Client for ROS2 controller manager
        self.switch_controller_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller'
        )
        
        # Wait for controller manager service
        while not self.switch_controller_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('⏳ Waiting for controller manager...')
        
        self.get_logger().info("🔄 Controller Switcher initialized")

    def handle_controller_switch(self, request, response):
        """Handle controller switching requests"""
        target_controller = request.controller_name
        self.get_logger().info(f"🔄 Switching to: {target_controller}")
        
        if target_controller not in self.available_controllers:
            response.success = False
            response.message = f"❌ Unknown controller: {target_controller}"
            return response
        
        # Create switch request
        switch_request = SwitchController.Request()
        switch_request.start_controllers = [target_controller]
        
        # Stop all other controllers
        for controller in self.available_controllers:
            if controller != target_controller:
                switch_request.stop_controllers.append(controller)
        
        switch_request.strictness = 2  # STRICT mode
        
        # Call controller manager service
        future = self.switch_controller_client.call_async(switch_request)
        
        # Wait for response
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            result = future.result()
            if result.ok:
                response.success = True
                response.message = f"✅ Successfully switched to {target_controller}"
                self.get_logger().info(f"✅ Active controller: {target_controller}")
            else:
                response.success = False
                response.message = f"❌ Failed to switch to {target_controller}"
        else:
            response.success = False
            response.message = "❌ Controller switch timeout"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    switcher = ControllerSwitcher()
    try:
        rclpy.spin(switcher)
    except KeyboardInterrupt:
        pass
    finally:
        switcher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
