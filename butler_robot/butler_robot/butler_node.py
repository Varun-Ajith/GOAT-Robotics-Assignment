#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
import json
import time

class RobotState(Enum):
    HOME = "home"
    MOVING_TO_KITCHEN = "moving_to_kitchen"
    AT_KITCHEN = "at_kitchen"
    MOVING_TO_TABLE = "moving_to_table"
    AT_TABLE = "at_table"
    RETURNING_HOME = "returning_home"

class ButlerNode(Node):
    def __init__(self):
        super().__init__('butler_node')
        self.state = RobotState.HOME
        self.status_pub_ = self.create_publisher(String, 'robot_status', 10)
        self.order_pub_ = self.create_subscription(String, 'new_order', self.order_callback, 10)
        
        self.movement_timer = None
        self.current_table = None

        self.get_logger().info('Butler Robot Node has been started')
    
    def publish_status(self, status_msg):
        msg = String()
        msg.data = status_msg
        self.status_pub_.publish(msg)
        self.get_logger().info(f"Robot status : {status_msg}")
    
    def simulate_movement(self, duration = 2.0):
        time.sleep(duration)
    
    def move_to_kitchen(self):
        self.state = RobotState.MOVING_TO_KITCHEN
        self.publish_status(f"Moving to kitchen")
        self.simulate_movement()
        self.state = RobotState.AT_KITCHEN
        self.publish_status(f"Arrived at kitchen")
    
    def move_to_table(self, table_number):
        self.state = RobotState.MOVING_TO_TABLE
        self.publish_status(f"moving to table {table_number}")
        self.simulate_movement()
        self.state = RobotState.AT_TABLE
        self.publish_status(f"Arrived at table {table_number}")
    
    def return_home(self):
        self.state = RobotState.RETURNING_HOME
        self.publish_status(f"Returning to home position")
        self.simulate_movement()
        self.state = RobotState.HOME
        self.publish_status(f"Arrived at home position")

    def order_callback(self, msg):
        try:
            order_data = json.loads(msg.data)
            table_number = order_data.get('table_number')

            if not table_number:
                self.get_logger().error("Invalid order format: missing table number")
                return
            self.get_logger().info(f"Processing order for table {table_number}")

            if self.state != RobotState.HOME:
                self.get_logger().warn("Robot is busy, cannot process new order at the moment")
                return

            self.current_table = table_number
            self.move_to_kitchen()
            self.move_to_table(table_number)
            self.return_home()
            self.current_table = None
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON format in order message")
        except Exception as e:
            self.get_logger().error(f"Error processing order : {str(e)}")

def main(args = None):
    rclpy.init(args = args)
    node = ButlerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()