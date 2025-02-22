#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import sys
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class TestButlerRobot(Node):
    def __init__(self):
        super().__init__('test_butler_robot')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Publisher for confirmations
        self.confirmation_pub = self.create_publisher(
            String, 
            'confirmation', 
            10
        )
        
        # Publisher for orders
        self.order_pub = self.create_publisher(
            String,
            'new_order',
            10
        )
        
        # Subscribe to robot status
        self.status_sub = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.last_status = None
        self.status_received = False
        
    def status_callback(self, msg):
        self.last_status = msg.data
        self.status_received = True
        self.get_logger().info(f'Robot Status: {msg.data}')
    
    def wait_for_status(self, timeout=10):
        """Wait for status message with timeout"""
        start_time = time.time()
        while not self.status_received and time.time() - start_time < timeout:
            time.sleep(0.1)
        if not self.status_received:
            self.get_logger().error('Timeout waiting for status message')
            return False
        return True
    
    def send_confirmation(self, location, confirmed=True):
        msg = String()
        msg.data = json.dumps({
            'location': location,
            'confirmed': confirmed
        })
        self.confirmation_pub.publish(msg)
        time.sleep(1)
    
    def send_order(self, table_number=None, table_numbers=None):
        msg = String()
        if table_number is not None:
            msg.data = json.dumps({'table_number': table_number})
        elif table_numbers is not None:
            msg.data = json.dumps({'table_numbers': table_numbers})
        self.order_pub.publish(msg)
        # Reset status flag after sending new order
        self.status_received = False
    
    def send_cancel(self, table_number):
        msg = String()
        msg.data = json.dumps({
            'table_number': table_number,
            'action': 'cancel'
        })
        self.order_pub.publish(msg)
        self.status_received = False
    
    def test_milestone_1(self):
        """Test single order delivery without confirmation"""
        self.get_logger().info('\nTesting Milestone 1: Single order delivery')
        
        self.send_order(table_number=1)
        time.sleep(2)  # Wait for order to be processed
        
        # Wait for completion status
        end_time = time.time() + 10  # 10 seconds timeout
        while time.time() < end_time:
            if self.last_status and 'All deliveries completed' in self.last_status:
                return True
            time.sleep(0.1)
        
        self.get_logger().error('Milestone 1 test timed out')
        return False
    
    def test_milestone_2(self):
        """Test single order delivery with confirmation"""
        self.get_logger().info('\nTesting Milestone 2: Single order with confirmation')
        
        self.send_order(table_number=1)
        if not self.wait_for_status():
            return False
        
        self.send_confirmation('kitchen')
        time.sleep(3)
        
        self.send_confirmation('table')
        time.sleep(3)
        
        return self.last_status and 'All deliveries completed' in self.last_status
    
    def test_milestone_3(self):
        """Test order with timeout scenarios"""
        self.get_logger().info('\nTesting Milestone 3: Order with timeouts')
        
        # Test kitchen timeout
        self.send_order(table_number=1)
        time.sleep(35)
        
        if not self.last_status or 'Timeout at kitchen' not in self.last_status:
            return False
        
        time.sleep(2)  # Wait for robot to return home
        
        # Test table timeout
        self.send_order(table_number=1)
        if not self.wait_for_status():
            return False
        
        self.send_confirmation('kitchen')
        time.sleep(35)
        
        return self.last_status and 'Timeout at table' in self.last_status
    
    def test_milestone_4(self):
        """Test order cancellation"""
        self.get_logger().info('\nTesting Milestone 4: Order cancellation')
        
        self.send_order(table_number=1)
        if not self.wait_for_status():
            return False
        
        self.send_cancel(table_number=1)
        time.sleep(3)
        
        return self.last_status and 'Order canceled' in self.last_status
    
    def test_milestone_5(self):
        """Test multiple order delivery"""
        self.get_logger().info('\nTesting Milestone 5: Multiple order delivery')
        
        self.send_order(table_numbers=[1, 2, 3])
        if not self.wait_for_status():
            return False
        
        self.send_confirmation('kitchen')
        time.sleep(3)
        
        for _ in range(3):
            self.send_confirmation('table')
            time.sleep(3)
        
        return self.last_status and 'All deliveries completed' in self.last_status

def main(args=None):
    rclpy.init(args=args)
    
    test_node = TestButlerRobot()
    executor = MultiThreadedExecutor()
    executor.add_node(test_node)
    
    # Start the executor in a separate thread
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # Get milestone number from command line argument
        milestone = int(sys.argv[1]) if len(sys.argv) > 1 else 0
        
        result = False
        if milestone == 1:
            result = test_node.test_milestone_1()
        elif milestone == 2:
            result = test_node.test_milestone_2()
        elif milestone == 3:
            result = test_node.test_milestone_3()
        elif milestone == 4:
            result = test_node.test_milestone_4()
        elif milestone == 5:
            result = test_node.test_milestone_5()
        else:
            test_node.get_logger().error('Invalid milestone number (1-5)')
            
        test_node.get_logger().info(f'Milestone {milestone} test {"passed" if result else "failed"}')
        
    finally:
        executor.shutdown()
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()