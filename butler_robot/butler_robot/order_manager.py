#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
import json
import time
import threading

class RobotState(Enum):
    HOME = "home"
    MOVING_TO_KITCHEN = "moving_to_kitchen"
    AT_KITCHEN = "at_kitchen"
    MOVING_TO_TABLE = "moving_to_table"
    AT_TABLE = "at_table"
    RETURNING_HOME = "returning_home"
    WAITING_KITCHEN = "waiting_kitchen"
    WAITING_TABLE = "waiting_table"

class ButlerNode(Node):
    def __init__(self):
        super().__init__('butler_node')
        self.state = RobotState.HOME
        
        # Publishers
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        
        # Subscribers
        self.order_sub = self.create_subscription(
            String,
            'new_order',
            self.order_callback,
            10)
        
        self.confirmation_sub = self.create_subscription(
            String,
            'confirmation',
            self.confirmation_callback,
            10)

        # Timeout settings
        self.TIMEOUT_DURATION = 30.0  # 30 seconds timeout
        self.waiting_timer = None
        self.confirmation_received = False
        
        self.get_logger().info('Butler Robot Node has been started')

    def publish_status(self, status_msg):
        msg = String()
        msg.data = status_msg
        self.status_pub.publish(msg)
        self.get_logger().info(f'Robot Status: {status_msg}')

    def simulate_movement(self, duration=2.0):
        time.sleep(duration)

    def start_timeout_timer(self):
        self.confirmation_received = False
        self.waiting_timer = threading.Timer(self.TIMEOUT_DURATION, self.handle_timeout)
        self.waiting_timer.start()

    def cancel_timeout_timer(self):
        if self.waiting_timer:
            self.waiting_timer.cancel()
            self.waiting_timer = None

    def handle_timeout(self):
        self.get_logger().warn('Timeout occurred while waiting for confirmation')
        if self.state == RobotState.WAITING_KITCHEN:
            self.publish_status("Timeout at kitchen. Returning home")
            self.return_home()
        elif self.state == RobotState.WAITING_TABLE:
            self.publish_status("Timeout at table. Returning to kitchen")
            self.move_to_kitchen()
            self.publish_status("Returning home")
            self.return_home()

    def confirmation_callback(self, msg):
        try:
            data = json.loads(msg.data)
            location = data.get('location')
            confirmed = data.get('confirmed', False)

            if confirmed:
                self.confirmation_received = True
                self.cancel_timeout_timer()
                
                if location == 'kitchen' and self.state == RobotState.WAITING_KITCHEN:
                    self.get_logger().info('Kitchen confirmation received')
                    self.move_to_table(self.current_table)
                elif location == 'table' and self.state == RobotState.WAITING_TABLE:
                    self.get_logger().info('Table confirmation received')
                    self.return_home()

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON format in confirmation message')

    def move_to_kitchen(self):
        self.state = RobotState.MOVING_TO_KITCHEN
        self.publish_status(f"Moving to kitchen")
        self.simulate_movement()
        self.state = RobotState.WAITING_KITCHEN
        self.publish_status(f"Waiting at kitchen for confirmation")
        self.start_timeout_timer()

    def move_to_table(self, table_number):
        self.state = RobotState.MOVING_TO_TABLE
        self.publish_status(f"Moving to table {table_number}")
        self.simulate_movement()
        self.state = RobotState.WAITING_TABLE
        self.publish_status(f"Waiting at table {table_number} for confirmation")
        self.start_timeout_timer()

    def return_home(self):
        self.state = RobotState.RETURNING_HOME
        self.publish_status("Returning to home position")
        self.simulate_movement()
        self.state = RobotState.HOME
        self.publish_status("Arrived at home position")

    def order_callback(self, msg):
        try:
            order_data = json.loads(msg.data)
            table_number = order_data.get('table_number')
            
            if not table_number:
                self.get_logger().error('Invalid order format: missing table number')
                return

            self.get_logger().info(f'Processing order for table {table_number}')
            
            if self.state != RobotState.HOME:
                self.get_logger().warn('Robot is busy, cannot process new order')
                return

            self.current_table = table_number
            self.move_to_kitchen()
            
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON format in order message')
        except Exception as e:
            self.get_logger().error(f'Error processing order: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ButlerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()