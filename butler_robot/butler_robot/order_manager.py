#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from enum import Enum
import threading
import time
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
        
        # Use callback group for handling multiple callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Publisher for new orders
        self.order_pub = self.create_publisher(
            String, 
            'new_order', 
            10
        )
        
        # Subscriber for robot status
        self.status_sub = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Initialize order tracking
        self.current_orders = {}  # Dictionary to track orders
        self.robot_status = "home"
        
        # Create service timer for checking order status
        self.create_timer(
            1.0,  # 1 second
            self.check_orders_status,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Order Manager Node has been started')

    def status_callback(self, msg):
        """Handle robot status updates"""
        self.robot_status = msg.data
        self.get_logger().info(f'Robot Status: {msg.data}')
        
        # Update order states based on robot status
        for order_id, order in self.current_orders.items():
            if order['state'] == OrderState.PENDING and "Moving to kitchen" in msg.data:
                order['state'] = OrderState.IN_PROGRESS
            elif order['state'] == OrderState.IN_PROGRESS and f"Arrived at table {order['table_number']}" in msg.data:
                order['state'] = OrderState.COMPLETED
            elif "Arrived at home position" in msg.data:
                if order['state'] == OrderState.IN_PROGRESS:
                    order['state'] = OrderState.COMPLETED

    def check_orders_status(self):
        """Periodically check the status of orders"""
        current_time = time.time()
        orders_to_remove = []

        for order_id, order in self.current_orders.items():
            # Check for timeout (5 minutes)
            if current_time - order['timestamp'] > 300:  # 5 minutes timeout
                if order['state'] != OrderState.COMPLETED:
                    order['state'] = OrderState.FAILED
                    self.get_logger().warn(f'Order {order_id} for table {order["table_number"]} timed out')
                orders_to_remove.append(order_id)
            
            # Remove completed orders after 1 minute
            elif order['state'] == OrderState.COMPLETED and current_time - order['timestamp'] > 60:
                orders_to_remove.append(order_id)

        # Remove processed orders
        for order_id in orders_to_remove:
            del self.current_orders[order_id]

    def send_order(self, table_number):
        """Send a new order to the robot"""
        try:
            # Create order data
            order_data = {
                'table_number': table_number,
                'timestamp': time.time()
            }
            
            # Create order tracking entry
            order_id = f"order_{time.time()}"
            self.current_orders[order_id] = {
                'table_number': table_number,
                'state': OrderState.PENDING,
                'timestamp': time.time()
            }
            
            # Publish order
            msg = String()
            msg.data = json.dumps(order_data)
            self.order_pub.publish(msg)
            
            self.get_logger().info(f'Sent order for table {table_number}')
            return order_id
            
        except Exception as e:
            self.get_logger().error(f'Error sending order: {str(e)}')
            return None

    def send_multiple_orders(self, table_numbers):
        """Send multiple orders"""
        order_ids = []
        for table in table_numbers:
            order_id = self.send_order(table)
            if order_id:
                order_ids.append(order_id)
            time.sleep(0.5)  # Small delay between orders
        return order_ids

    def cancel_order(self, table_number):
        """Cancel an order for a specific table"""
        try:
            cancel_data = {
                'table_number': table_number,
                'action': 'cancel'
            }
            
            msg = String()
            msg.data = json.dumps(cancel_data)
            self.order_pub.publish(msg)
            
            # Update order state for the cancelled table
            for order_id, order in self.current_orders.items():
                if order['table_number'] == table_number and order['state'] != OrderState.COMPLETED:
                    order['state'] = OrderState.FAILED
                    self.get_logger().info(f'Order for table {table_number} has been cancelled')
            
        except Exception as e:
            self.get_logger().error(f'Error cancelling order: {str(e)}')

    def get_order_status(self, order_id):
        """Get the status of a specific order"""
        if order_id in self.current_orders:
            return self.current_orders[order_id]['state']
        return None

def main(args=None):
    rclpy.init(args=args)
    
    # Create and start the node with MultiThreadedExecutor
    node = OrderManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # Spin the node
        executor.spin()
    finally:
        # Cleanup
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()