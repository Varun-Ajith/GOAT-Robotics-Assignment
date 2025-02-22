#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import SetBool
import json
import time
from enum import Enum
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class OrderState(Enum):
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"


class OrderManager(Node):
    def __init__(self):
        super().__init__('order_manager')
        self.callback_group = ReentrantCallbackGroup()

        # Publishers and Subscribers
        self.order_pub = self.create_publisher(String, 'new_order', 10)
        self.status_sub = self.create_subscription(
            String, 'robot_status', self.status_callback, 10,
            callback_group=self.callback_group
        )

        # Services
        self.send_order_srv = self.create_service(
            SetBool, 'send_order', self.send_order_callback,
            callback_group=self.callback_group
        )
        self.send_multiple_orders_srv = self.create_service(
            SetBool, 'send_multiple_orders', self.send_multiple_orders_callback,
            callback_group=self.callback_group
        )
        self.cancel_order_srv = self.create_service(
            SetBool, 'cancel_order', self.cancel_order_callback,
            callback_group=self.callback_group
        )

        # State tracking
        self.current_orders = {}
        self.robot_status = "home"

        # Timer for order status checking
        self.create_timer(1.0, self.check_orders_status, callback_group=self.callback_group)

        self.get_logger().info('Order Manager Node has been started')

    def status_callback(self, msg):
        self.robot_status = msg.data
        self.get_logger().info(f'Robot Status: {msg.data}')

        for order_id, order in self.current_orders.items():
            if order['state'] == OrderState.PENDING and "Moving to kitchen" in msg.data:
                order['state'] = OrderState.IN_PROGRESS
            elif order['state'] == OrderState.IN_PROGRESS and "All deliveries completed" in msg.data:
                order['state'] = OrderState.COMPLETED

    def send_order_callback(self, request, response):
        try:
            data = json.loads(request.data)
            table_number = data.get('table_number')
            if table_number:
                order_id = self.send_order(table_number)
                response.success = True
                response.message = str(order_id)
            else:
                response.success = False
                response.message = "Invalid table number"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def send_multiple_orders_callback(self, request, response):
        try:
            data = json.loads(request.data)
            table_numbers = data.get('table_numbers')
            if table_numbers:
                order_id = self.send_multiple_orders(table_numbers)
                response.success = True
                response.message = str(order_id)
            else:
                response.success = False
                response.message = "Invalid table numbers"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def cancel_order_callback(self, request, response):
        try:
            data = json.loads(request.data)
            table_number = data.get('table_number')
            if table_number:
                self.cancel_order(table_number)
                response.success = True
                response.message = f"Cancelled order for table {table_number}"
            else:
                response.success = False
                response.message = "Invalid table number"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def check_orders_status(self):
        current_time = time.time()
        orders_to_remove = []

        for order_id, order in self.current_orders.items():
            if current_time - order['timestamp'] > 300:  # 5 minutes timeout
                if order['state'] != OrderState.COMPLETED:
                    order['state'] = OrderState.FAILED
                    table_info = order.get('table_numbers', [order.get('table_number')])
                    self.get_logger().warn(f'Order {order_id} for tables {table_info} timed out')
                orders_to_remove.append(order_id)
            elif order['state'] == OrderState.COMPLETED and current_time - order['timestamp'] > 60:
                orders_to_remove.append(order_id)

        for order_id in orders_to_remove:
            del self.current_orders[order_id]

    def send_multiple_orders(self, table_numbers):
        try:
            order_data = {
                'table_numbers': table_numbers,
                'timestamp': time.time()
            }

            order_id = f"batch_order_{time.time()}"
            self.current_orders[order_id] = {
                'table_numbers': table_numbers,
                'state': OrderState.PENDING,
                'timestamp': time.time()
            }

            msg = String()
            msg.data = json.dumps(order_data)
            self.order_pub.publish(msg)

            self.get_logger().info(f'Sent batch order for tables {table_numbers}')
            return order_id

        except Exception as e:
            self.get_logger().error(f'Error sending batch order: {str(e)}')
            return None

    def send_order(self, table_number):
        try:
            order_data = {
                'table_number': table_number,
                'timestamp': time.time()
            }

            order_id = f"order_{time.time()}"
            self.current_orders[order_id] = {
                'table_number': table_number,
                'state': OrderState.PENDING,
                'timestamp': time.time()
            }

            msg = String()
            msg.data = json.dumps(order_data)
            self.order_pub.publish(msg)

            self.get_logger().info(f'Sent order for table {table_number}')
            return order_id

        except Exception as e:
            self.get_logger().error(f'Error sending order: {str(e)}')
            return None

    def cancel_order(self, table_number):
        try:
            cancel_data = {
                'table_number': table_number,
                'action': 'cancel'
            }

            msg = String()
            msg.data = json.dumps(cancel_data)
            self.order_pub.publish(msg)

            for order_id, order in self.current_orders.items():
                tables = order.get('table_numbers', [order.get('table_number')])
                if table_number in tables and order['state'] != OrderState.COMPLETED:
                    order['state'] = OrderState.FAILED
                    self.get_logger().info(f'Order for table {table_number} has been cancelled')

        except Exception as e:
            self.get_logger().error(f'Error cancelling order: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = OrderManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
