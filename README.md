# GOAT Robotics Assignment - ROS Developer

## Problem Statement

The French Door Café has requested a robot butler to handle food delivery tasks. The robot must manage orders for 3 tables (table1, table2, table3) and handle various scenarios, including:

1. Single order delivery without confirmation.
2. Single order delivery with confirmation.
3. Handling timeouts at the kitchen or table.
4. Order cancellation during delivery.
5. Multiple order delivery with confirmation.

The robot must efficiently move between the home position, kitchen, and customer tables, ensuring timely delivery and handling edge cases like timeouts and cancellations.

## Solution Overview

The solution consists of three ROS nodes:

1. **Butler Node (`butler_node.py`)**: Manages the robot's state and movement logic. It handles orders, timeouts, cancellations, and confirmations.
2. **Order Manager Node (`order_manager.py`)**: Manages incoming orders, tracks their status, and communicates with the butler node.
3. **Test Utilities Node (`test_utils.py`)**: Provides automated testing for all milestones (1 to 5).

### Key Features:
- **State Management**: The robot's state is managed using an `Enum` (`RobotState`), ensuring clear transitions between states like `HOME`, `MOVING_TO_KITCHEN`, `AT_TABLE`, etc.
- **Timeout Handling**: The robot handles timeouts at the kitchen or table and takes appropriate actions (e.g., returning home or moving to the next table).
- **Order Cancellation**: The robot can cancel orders mid-delivery and return to the kitchen or home position based on the current state.
- **Multiple Order Delivery**: The robot can handle multiple orders sequentially, ensuring all tables are served before returning home.

## Code Structure

### Workspace and Package
- **Workspace Name**: `goat_ws`
- **Package Name**: `butler_robot`
- **GitHub Repo**: [GOAT-Robotics-Assignment](https://github.com/Varun-Ajith/GOAT-Robotics-Assignment)

### Nodes
1. **Butler Node** (`butler_node.py`):
   - Manages the robot's state and movement logic.
   - Handles orders, timeouts, cancellations, and confirmations.
   - Publishes robot status updates.

2. **Order Manager Node** (`order_manager.py`):
   - Manages incoming orders and tracks their status.
   - Communicates with the butler node to process orders.

3. **Test Utilities Node** (`test_utils.py`):
   - Provides automated testing for all milestones (1 to 5).
   - Simulates order requests, confirmations, and cancellations.

### Topics and Services
- **Topics**:
  - `/new_order`: Publishes new orders to the butler node.
  - `/robot_status`: Publishes the robot's current status.
  - `/confirmation`: Publishes confirmations from the kitchen or table.
  - `/cancel_order`: Publishes order cancellation requests.

- **Services**:
  - `/send_order`: Service to send a single order.
  - `/send_multiple_orders`: Service to send multiple orders.
  - `/cancel_order`: Service to cancel an order.

## Milestones Achieved

The solution satisfies all 5 milestones:

| Milestone | Aim | Test Command | Additional Commands/Messages Needed |
|-----------|-----|--------------|-------------------------------------|
| 1 | Basic single order delivery - Robot moves from home to kitchen to table and back | `ros2 run butler_robot test_utils 1` | None - fully automated |
| 2 | Single order with confirmations - Tests if robot waits for confirmations at kitchen and table | `ros2 run butler_robot test_utils 2` | When robot status shows "Waiting at kitchen", in a new terminal send kitchen confirmation:<br>`ros2 topic pub /confirmation std_msgs/String "data: '{\"location\":\"kitchen\", \"confirmed\":true}'"`<br>When robot status shows "Waiting at table", send table confirmation:<br>`ros2 topic pub /confirmation std_msgs/String "data: '{\"location\":\"table\", \"confirmed\":true}'"` |
| 3 | Timeout handling - Tests robot's behavior when no confirmation is received | `ros2 run butler_robot test_utils 3` | No commands needed - test will wait for timeouts |
| 4 | Order cancellation - Tests if robot properly handles order cancellation | `ros2 run butler_robot test_utils 4` | To manually cancel an order:<br>`ros2 topic pub /new_order std_msgs/String "data: '{\"table_number\":1,\"action\":\"cancel\"}'"` |
| 5 | Multiple order delivery - Tests if robot can handle multiple table deliveries | `ros2 run butler_robot test_utils 5` | When robot status shows "Waiting at kitchen", in a new terminal send kitchen confirmation:<br>`ros2 topic pub /confirmation std_msgs/String "data: '{\"location\":\"kitchen\", \"confirmed\":true}'"`<br>For each table, wait for "Waiting at table" status and send table confirmation:<br>`ros2 topic pub /confirmation std_msgs/String "data: '{\"location\":\"table\", \"confirmed\":true}'"` |

> **Note:** Milestone 2 and Milestone 5 require manual confirmation inputs at the kitchen and table stages, so they are not fully automated. Ensure you monitor the robot’s status and send the necessary confirmation messages when prompted.

## How to Run the Code

### Prerequisites
- ROS 2 (tested on Jazzy).
- Python 3.

### Steps to Run:
1. Clone the repository:
   ```bash
   git clone https://github.com/Varun-Ajith/GOAT-Robotics-Assignment.git
   cd GOAT-Robotics-Assignment/goat_ws
   ```

2. Build the workspace:
   ```bash
   colcon build
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

4. Run the nodes:
   - Start the **Butler Node**:
     ```bash
     ros2 run butler_robot butler_node
     ```
   - Start the **Order Manager Node**:
     ```bash
     ros2 run butler_robot order_manager
     ```
   - Run the **Test Utilities Node** for a specific milestone (e.g., Milestone 1):
     ```bash
     ros2 run butler_robot test_utils 1
     ```

### Testing Milestones
- To test a specific milestone, run the `test_utils` node with the milestone number as an argument:
  ```bash
  ros2 run butler_robot test_utils <milestone_number>
  ```
  Replace `<milestone_number>` with `1`, `2`, `3`, `4`, or `5`.
  
### Screen Recording

For a demonstration of the robot’s performance and behavior across all milestones, please refer to the screen recording provided. This recording captures the robot’s navigation, order handling, and response to confirmations and cancellations.

|**Milestone1**|**Milestone2**|**Milestone3**|**Milestone4**|**Milestone5**|
|--------------|--------------|--------------|--------------|--------------|
|[Milestone-1](medias/Milestone1.gif) | [Milestone-2](medias/Milestone2.gif) | [Milestone-3](medias/Milestone3.gif) | [Milestone-5](medias/Milestone4.gif) | [Milestone-5](medias/Milestone5.gif) |

## Documentation and Approach

### Approach
- **State Machine**: The robot's behavior is modeled as a state machine, ensuring clear transitions between states.
- **Modular Design**: The code is modular, with separate nodes for order management and robot control.
- **Timeout Handling**: Timeouts are handled using a `threading.Timer`, ensuring the robot can recover from unresponsive scenarios.
- **Cancellation Logic**: The robot can cancel orders mid-delivery and return to the kitchen or home position based on the current state.

### Code Documentation
- The code is well-documented with comments explaining key logic and functionality.
- Each node has a clear purpose and follows ROS 2 best practices.

