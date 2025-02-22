#!/usr/bin/env python3

import json
import threading
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotState(Enum):
    HOME = "home"
    MOVING_TO_KITCHEN = "moving_to_kitchen"
    AT_KITCHEN = "at_kitchen"
    MOVING_TO_TABLE = "moving_to_table"
    AT_TABLE = "at_table"
    RETURNING_HOME = "returning_home"
    WAITING_KITCHEN = "waiting_kitchen"
    WAITING_TABLE = "waiting_table"
    RETURNING_TO_KITCHEN = "returning_to_kitchen"
    CANCELED = "canceled"


class ButlerNode(Node):
    TIMEOUT_DURATION = 30.0

    def __init__(self):
        super().__init__('butler_node')
        self.state = RobotState.HOME

        # Publishers
        self.status_pub = self.create_publisher(String, 'robot_status', 10)

        # Subscribers
        self.create_subscription(String, 'new_order', self.order_callback, 10)
        self.create_subscription(String, 'confirmation', self.confirmation_callback, 10)
        self.create_subscription(String, 'cancel_order', self.cancel_callback, 10)

        # Initialize variables
        self.waiting_timer = None
        self.confirmation_received = False
        self.current_table = None
        self.has_food = False
        self.is_canceled = False
        self.order_queue = []
        self.remaining_tables = []

        self.get_logger().info('Butler Robot Node has been started')

    def publish_status(self, status_msg):
        msg = String()
        msg.data = status_msg
        self.status_pub.publish(msg)
        self.get_logger().info(f'Robot Status: {status_msg}')

    def simulate_movement(self, duration=2.0):
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.is_canceled:
                return False
            time.sleep(0.1)
        return True

    def cancel_callback(self, msg):
        try:
            data = json.loads(msg.data)
            table_number = data.get('table_number')

            if table_number == self.current_table:
                self.get_logger().info(f'Cancellation received for table {table_number}')
                self.is_canceled = True
                self.cancel_timeout_timer()

                if self.state in [RobotState.MOVING_TO_TABLE, RobotState.WAITING_TABLE]:
                    self.publish_status(f"Order canceled while serving table {table_number}. Returning to kitchen")
                    self.return_to_kitchen_then_home()
                elif self.state in [RobotState.MOVING_TO_KITCHEN, RobotState.WAITING_KITCHEN]:
                    self.publish_status("Order canceled while going to kitchen. Returning home")
                    self.return_home()

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON format in cancel message')

    def start_timeout_timer(self):
        self.confirmation_received = False
        self.waiting_timer = threading.Timer(self.TIMEOUT_DURATION, self.handle_timeout)
        self.waiting_timer.start()

    def cancel_timeout_timer(self):
        if self.waiting_timer:
            self.waiting_timer.cancel()
            self.waiting_timer = None

    def handle_timeout(self):
        if self.is_canceled:
            return

        self.get_logger().warn('Timeout occurred while waiting for confirmation')

        if self.state == RobotState.WAITING_KITCHEN:
            self.publish_status("Timeout at kitchen. No food collected. Returning home")
            self.has_food = False
            self.return_home()
        elif self.state == RobotState.WAITING_TABLE:
            if self.remaining_tables:
                self.current_table = self.remaining_tables.pop(0)
                self.move_to_table(self.current_table)
            else:
                self.publish_status("Timeout at table. Returning to kitchen first")
                self.return_to_kitchen_then_home()

    def return_to_kitchen_then_home(self):
        if not self.is_canceled:
            self.state = RobotState.RETURNING_TO_KITCHEN
            self.publish_status("Returning to kitchen")
            if not self.simulate_movement():
                return

        self.state = RobotState.AT_KITCHEN
        self.publish_status("Arrived at kitchen")
        self.has_food = False
        self.return_home()

    def confirmation_callback(self, msg):
        if self.is_canceled:
            return

        try:
            data = json.loads(msg.data)
            location = data.get('location')
            confirmed = data.get('confirmed', False)

            if confirmed:
                self.confirmation_received = True
                self.cancel_timeout_timer()

                if location == 'kitchen' and self.state == RobotState.WAITING_KITCHEN:
                    self.get_logger().info('Kitchen confirmation received')
                    self.has_food = True
                    self.process_next_table()
                elif location == 'table' and self.state == RobotState.WAITING_TABLE:
                    self.get_logger().info('Table confirmation received')
                    self.process_next_table()

        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON format in confirmation message')

    def process_next_table(self):
        if self.remaining_tables:
            self.current_table = self.remaining_tables.pop(0)
            self.move_to_table(self.current_table)
        else:
            self.has_food = False
            self.return_to_kitchen_then_home()

    def move_to_kitchen(self):
        self.state = RobotState.MOVING_TO_KITCHEN
        self.publish_status(f"Moving to kitchen to collect orders for tables: {[self.current_table] + self.remaining_tables}")
        if not self.simulate_movement():
            return

        self.state = RobotState.WAITING_KITCHEN
        self.publish_status("Waiting at kitchen for confirmation")
        self.start_timeout_timer()

    def move_to_table(self, table_number):
        if self.is_canceled:
            return

        self.state = RobotState.MOVING_TO_TABLE
        self.publish_status(f"Moving to table {table_number}")
        if not self.simulate_movement():
            return

        self.state = RobotState.WAITING_TABLE
        self.publish_status(f"Waiting at table {table_number} for confirmation")
        self.start_timeout_timer()

    def return_home(self):
        self.state = RobotState.RETURNING_HOME
        self.publish_status("Returning to home position")
        self.simulate_movement()
        self.reset_robot_state()

    def reset_robot_state(self):
        self.state = RobotState.HOME
        self.current_table = None
        self.has_food = False
        self.is_canceled = False
        self.remaining_tables = []
        self.publish_status("All deliveries completed. Arrived at home position")

    def order_callback(self, msg):
        try:
            order_data = json.loads(msg.data)
            table_numbers = order_data.get('table_numbers', [order_data.get('table_number')])

            if not table_numbers:
                self.get_logger().error('Invalid order format: missing table numbers')
                return

            if self.state != RobotState.HOME:
                self.get_logger().warn('Robot is busy, cannot process new orders')
                return

            self.get_logger().info(f'Processing orders for tables: {table_numbers}')
            self.current_table = table_numbers[0]
            self.remaining_tables = table_numbers[1:]
            self.is_canceled = False
            self.move_to_kitchen()

        except (json.JSONDecodeError, Exception) as e:
            self.get_logger().error(f'Error processing order: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ButlerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
