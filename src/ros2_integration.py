
#!/usr/bin/env python3

"""
ROS2 Integration Module for Dijkstra Path Planning Robot

This module demonstrates how to integrate the Dijkstra path planning
algorithm with ROS2 Humble, RViz, and Gazebo. It serves as a template
and documentation for the full ROS2 implementation.

Requirements:
- ROS2 Humble installed
- Python 3.8+
- numpy, matplotlib

Note: This file contains pseudocode and documentation for ROS2 integration.
The actual implementation would require a proper ROS2 workspace setup.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import tf2_ros

# Import our Dijkstra implementation
from dijkstra_robot import DijkstraPathPlanner


class DijkstraROS2Node(Node):
    """ROS2 Node for Dijkstra Path Planning Robot"""
    
    def __init__(self):
        super().__init__('dijkstra_path_planner')
        
        # Create logger
        self.get_logger().info('Initializing Dijkstra Path Planner')
        
        # Initialize parameters
        self.declare_parameter('map_resolution', 0.1)  # meters per cell
        self.declare_parameter('robot_radius', 0.2)    # robot radius in meters
        
        # Create publishers
        self.path_publisher = self.create_publisher(
            Path, 
            'planned_path', 
            10
        )
        
        self.visualization_publisher = self.create_publisher(
            MarkerArray,
            'path_visualization',
            10
        )
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Create subscribers
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize map and planner
        self.map_data = None
        self.map_resolution = self.get_parameter('map_resolution').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.planner = None
        self.start_position = None
        self.goal_position = None
        self.current_path = None
        
        self.get_logger().info('Dijkstra Path Planner initialized')
    
    def map_callback(self, msg):
        """Process incoming occupancy grid map"""
        self.get_logger().info('Received map')
        
        # Convert ROS OccupancyGrid to numpy array for our planner
        width = msg.info.width
        height = msg.info.height
        self.map_data = np.array(msg.data).reshape(height, width)
        
        # Convert occupancy probabilities (0-100) to binary grid (0-1)
        grid = np.zeros((width, height))
        for i in range(width):
            for j in range(height):
                # Mark as obstacle if occupancy > 50 or if it's unknown (-1)
                if self.map_data[j, i] > 50 or self.map_data[j, i] == -1:
                    grid[i, j] = 1
        
        # Initialize planner with the new grid
        self.planner = DijkstraPathPlanner(grid_size=(width, height))
        self.planner.grid = grid
        
        self.get_logger().info('Map processed')
    
    def goal_callback(self, msg):
        """Process goal pose and plan a path"""
        self.get_logger().info('Received goal')
        
        if self.map_data is None or self.planner is None:
            self.get_logger().warn('No map data available yet')
            return
            
        # Get robot pose from tf
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            
            # Extract robot position from transform
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Convert to grid coordinates
            start_x = int(robot_x / self.map_resolution)
            start_y = int(robot_y / self.map_resolution)
            self.start_position = (start_x, start_y)
            
            self.get_logger().info(f'Robot at grid position: {self.start_position}')
        except Exception as e:
            self.get_logger().error(f'Failed to get robot position: {str(e)}')
            return
            
        # Extract goal position
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        # Convert to grid coordinates
        goal_x_grid = int(goal_x / self.map_resolution)
        goal_y_grid = int(goal_y / self.map_resolution)
        self.goal_position = (goal_x_grid, goal_y_grid)
        
        self.get_logger().info(f'Goal set at grid position: {self.goal_position}')
        
        # Set start and goal in planner
        self.planner.set_start(self.start_position)
        self.planner.set_goal(self.goal_position)
        
        # Run Dijkstra algorithm
        self.get_logger().info('Planning path...')
        path = self.planner.find_path(visualize=False)  # No matplotlib viz in ROS
        
        if path:
            self.get_logger().info(f'Path found with {len(path)} waypoints')
            self.current_path = path
            
            # Publish path for RViz
            self.publish_path(path)
            
            # Start following the path
            self.follow_path()
        else:
            self.get_logger().warn('No path found')
    
    def publish_path(self, path):
        """Publish path for visualization in RViz"""
        # Create ROS Path message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Add each point to the path
        for point in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0] * self.map_resolution
            pose.pose.position.y = point[1] * self.map_resolution
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        # Publish the path
        self.path_publisher.publish(path_msg)
        
        # Also publish visualization markers
        self.publish_visualization(path)
    
    def publish_visualization(self, path):
        """Publish marker array for path visualization"""
        marker_array = MarkerArray()
        
        # Path line marker
        line_marker = Marker()
        line_marker.header.frame_id = 'map'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'path_line'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05  # Line width
        line_marker.color.r = 0.0
        line_marker.color.g = 0.0
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0
        
        # Add points to line
        for point in path:
            p = geometry_msgs.msg.Point()
            p.x = point[0] * self.map_resolution
            p.y = point[1] * self.map_resolution
            p.z = 0.1  # Slightly above ground
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        # Publish markers
        self.visualization_publisher.publish(marker_array)
    
    def follow_path(self):
        """Follow the planned path - simple implementation"""
        self.get_logger().info('Following path...')
        
        # In a real implementation, this would be a more sophisticated
        # path following algorithm with a control loop. Here, we just
        # provide a simplified version that would move toward each waypoint.
        
        if not self.current_path:
            return
            
        # Create a timer to send velocity commands
        self.create_timer(0.1, self.path_following_callback)
    
    def path_following_callback(self):
        """Timer callback for path following"""
        if not self.current_path:
            self.get_logger().info('Path completed')
            
            # Send stop command
            cmd = Twist()
            self.cmd_vel_publisher.publish(cmd)
            return False
            
        # Get next waypoint
        next_point = self.current_path[0]
        
        # Convert to world coordinates
        target_x = next_point[0] * self.map_resolution
        target_y = next_point[1] * self.map_resolution
        
        # Get current robot position
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            
            # Calculate direction to target
            dx = target_x - current_x
            dy = target_y - current_y
            distance = np.sqrt(dx*dx + dy*dy)
            
            if distance < 0.1:  # If close enough to waypoint
                self.current_path.pop(0)  # Remove this waypoint
                return True
                
            # Calculate orientation to target
            target_yaw = np.arctan2(dy, dx)
            
            # Get current orientation
            q = transform.transform.rotation
            _, _, current_yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
            
            # Calculate angular difference
            yaw_diff = target_yaw - current_yaw
            while yaw_diff > np.pi:
                yaw_diff -= 2*np.pi
            while yaw_diff < -np.pi:
                yaw_diff += 2*np.pi
                
            # Create velocity command
            cmd = Twist()
            
            # Angular velocity proportional to the yaw difference
            cmd.angular.z = 0.5 * yaw_diff
            
            # Linear velocity proportional to distance, but reduced if turning
            cmd.linear.x = min(0.2, 0.3 * distance) * (1 - abs(yaw_diff) / np.pi)
            
            # Publish velocity command
            self.cmd_vel_publisher.publish(cmd)
            
        except Exception as e:
            self.get_logger().error(f'Path following error: {str(e)}')
            return False
            
        return True
    
    def euler_from_quaternion(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
            
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw


def main(args=None):
    """Main function"""
    print("Note: This is a template for ROS2 integration.")
    print("To run this, you need a proper ROS2 workspace setup.")
    print("The following code demonstrates how it would be run:")
    
    print("\n--- ROS2 Integration Code ---")
    print("rclpy.init(args=args)")
    print("node = DijkstraROS2Node()")
    print("rclpy.spin(node)")
    print("node.destroy_node()")
    print("rclpy.shutdown()")
    print("---------------------------\n")
    
    print("For a full ROS2 implementation, you would need to:")
    print("1. Set up a ROS2 workspace")
    print("2. Create a proper package structure")
    print("3. Add setup.py, package.xml, etc.")
    print("4. Build the package with colcon")
    print("5. Source the workspace")
    print("6. Run the node with 'ros2 run your_package dijkstra_node'")


if __name__ == "__main__":
    main()
