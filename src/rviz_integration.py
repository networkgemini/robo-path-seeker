
#!/usr/bin/env python3

"""
RViz Integration Module for Dijkstra Path Planning Robot

This module demonstrates how to visualize the Dijkstra path planning
algorithm in RViz. It includes:
1. Visualization of the occupancy grid (map)
2. Visualization of the planned path
3. Visualization of the robot position and orientation
4. Visualization of the exploration process

Requirements:
- ROS2 Humble installed
- RViz2 installed
- Python 3.8+

Note: This is a template with pseudocode for RViz integration.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA
import os

class RVizVisualization(Node):
    """Node for visualizing Dijkstra planning in RViz"""
    
    def __init__(self):
        super().__init__('rviz_visualization')
        
        # Publishers for visualization
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'visualization_markers', 10)
        self.visited_pub = self.create_publisher(OccupancyGrid, 'visited_cells', 10)
        self.frontier_pub = self.create_publisher(Marker, 'frontier', 10)
        
        # Initialize visualization elements
        self.marker_array = MarkerArray()
        self.next_marker_id = 0
        
        # Map params
        self.map_frame = 'map'
        self.map_resolution = 0.05  # meters per cell
        self.map_width = 200  # cells
        self.map_height = 200  # cells
        self.map_origin_x = -5.0  # meters
        self.map_origin_y = -5.0  # meters
        
        self.get_logger().info('RViz visualization node initialized')
    
    def visualize_grid(self, grid):
        """Visualize the occupancy grid"""
        # Create grid message
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = self.map_frame
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set metadata
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.origin.position.x = self.map_origin_x
        grid_msg.info.origin.position.y = self.map_origin_y
        grid_msg.info.origin.orientation.w = 1.0
        
        # Convert grid data
        # Assuming grid is a numpy array with 0 for free, 1 for obstacle
        data = []
        for i in range(self.map_height):
            for j in range(self.map_width):
                if j < grid.shape[1] and i < grid.shape[0]:
                    if grid[i, j] == 1:  # Obstacle
                        data.append(100)  # Occupied (100%)
                    else:
                        data.append(0)    # Free (0%)
                else:
                    data.append(-1)   # Unknown (-1)
        
        grid_msg.data = data
        
        # Publish to a topic that RViz can display
        # In a real implementation, this would be published to the 'map' topic
        self.get_logger().info('Grid visualization created')
        
        return grid_msg
    
    def visualize_path(self, path):
        """Visualize the planned path"""
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Convert path points to PoseStamped messages
        for point in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            
            # Convert grid coordinates to world coordinates
            pose.pose.position.x = self.map_origin_x + point[0] * self.map_resolution
            pose.pose.position.y = self.map_origin_y + point[1] * self.map_resolution
            pose.pose.position.z = 0.1  # Slightly above ground for visibility
            
            # Set orientation (pointing forward along path)
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        # Publish path
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path)} points')
        
        # Also create a line marker for better visualization
        self.create_path_marker(path)
    
    def create_path_marker(self, path):
        """Create a line marker for the path"""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'path'
        marker.id = self.next_marker_id
        self.next_marker_id += 1
        
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set marker properties
        marker.scale.x = 0.05  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Convert path points to marker points
        for point in path:
            p = Point()
            p.x = self.map_origin_x + point[0] * self.map_resolution
            p.y = self.map_origin_y + point[1] * self.map_resolution
            p.z = 0.05  # Slightly above ground
            marker.points.append(p)
        
        # Add to marker array and publish
        self.marker_array.markers.append(marker)
        self.markers_pub.publish(self.marker_array)
    
    def visualize_visited_cells(self, visited_cells):
        """Visualize cells visited during search"""
        # Create a grid showing visited cells
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = self.map_frame
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set metadata
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_width
        grid_msg.info.height = self.map_height
        grid_msg.info.origin.position.x = self.map_origin_x
        grid_msg.info.origin.position.y = self.map_origin_y
        grid_msg.info.origin.orientation.w = 1.0
        
        # Create data array
        data = np.zeros(self.map_width * self.map_height, dtype=np.int8)
        
        # Mark visited cells with a value (e.g. 50 for half-occupied)
        for cell in visited_cells:
            if 0 <= cell[0] < self.map_width and 0 <= cell[1] < self.map_height:
                idx = cell[1] * self.map_width + cell[0]
                data[idx] = 50  # Custom value for visualization
        
        grid_msg.data = data.tolist()
        
        # Publish
        self.visited_pub.publish(grid_msg)
    
    def visualize_frontier(self, frontier_cells):
        """Visualize frontier cells (next to be explored)"""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontier'
        marker.id = 0
        
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # Set marker properties
        marker.scale.x = 0.05  # Point size
        marker.scale.y = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Add points for each frontier cell
        for cell in frontier_cells:
            p = Point()
            p.x = self.map_origin_x + cell[0] * self.map_resolution
            p.y = self.map_origin_y + cell[1] * self.map_resolution
            p.z = 0.05
            marker.points.append(p)
        
        # Publish
        self.frontier_pub.publish(marker)
    
    def visualize_start_and_goal(self, start, goal):
        """Visualize start and goal positions"""
        # Create markers for start and goal
        start_marker = Marker()
        start_marker.header.frame_id = self.map_frame
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.ns = 'endpoints'
        start_marker.id = self.next_marker_id
        self.next_marker_id += 1
        
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        
        # Set marker properties
        start_marker.scale.x = 0.2
        start_marker.scale.y = 0.2
        start_marker.scale.z = 0.2
        
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 1.0
        
        # Set position
        start_marker.pose.position.x = self.map_origin_x + start[0] * self.map_resolution
        start_marker.pose.position.y = self.map_origin_y + start[1] * self.map_resolution
        start_marker.pose.position.z = 0.1
        start_marker.pose.orientation.w = 1.0
        
        # Goal marker
        goal_marker = Marker()
        goal_marker.header.frame_id = self.map_frame
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = 'endpoints'
        goal_marker.id = self.next_marker_id
        self.next_marker_id += 1
        
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        
        # Set marker properties
        goal_marker.scale.x = 0.2
        goal_marker.scale.y = 0.2
        goal_marker.scale.z = 0.2
        
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        
        # Set position
        goal_marker.pose.position.x = self.map_origin_x + goal[0] * self.map_resolution
        goal_marker.pose.position.y = self.map_origin_y + goal[1] * self.map_resolution
        goal_marker.pose.position.z = 0.1
        goal_marker.pose.orientation.w = 1.0
        
        # Add to marker array and publish
        self.marker_array.markers.append(start_marker)
        self.marker_array.markers.append(goal_marker)
        self.markers_pub.publish(self.marker_array)
    
    def create_rviz_config(self, output_path='rviz_config.rviz'):
        """Create an RViz configuration file"""
        # This would create an RViz config file with appropriate displays
        # In a real implementation, this would be a more sophisticated config
        
        config = """
displays:
  - Class: rviz_default_plugins/Grid
    Name: Grid
    Plane Cell Count: 10
    Normal: 0, 0, 1
    Offset: 0, 0, 0
    Plane: XY
    
  - Class: rviz_default_plugins/Map
    Name: Map
    Topic: /map
    Value: true
    
  - Class: rviz_default_plugins/Path
    Name: Path
    Topic: /planned_path
    Color: 25; 255; 0
    
  - Class: rviz_default_plugins/MarkerArray
    Name: MarkerArray
    Topic: /visualization_markers
    
  - Class: rviz_default_plugins/Map
    Name: Visited
    Topic: /visited_cells
    Color Scheme: map
    
  - Class: rviz_default_plugins/Marker
    Name: Frontier
    Topic: /frontier
"""
        
        # In a real implementation, you would write this to a file
        print(f"RViz configuration would be saved to {output_path}")
        print("Sample configuration:")
        print(config)
        
        return config


def create_launch_file():
    """
    This would create a launch file for running the system with RViz
    In a real implementation, this would be part of your package
    """
    launch_file = """
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'dijkstra_robot'
    
    # Get package directory
    pkg_dir = get_package_share_directory(package_name)
    
    # RViz configuration
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        
        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Dijkstra path planner node
        Node(
            package=package_name,
            executable='dijkstra_node',
            name='dijkstra_path_planner',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # RViz visualization node
        Node(
            package=package_name,
            executable='rviz_visualization',
            name='rviz_visualization',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
"""
    
    print("Sample launch file:")
    print(launch_file)
    
    return launch_file


def main(args=None):
    """Main function"""
    print("Note: This is a template for RViz integration.")
    print("To run this, you need a proper ROS2 workspace setup.")
    print("The following code demonstrates how it would be run:")
    
    print("\n--- RViz Integration Code ---")
    print("rclpy.init(args=args)")
    print("node = RVizVisualization()")
    print("node.create_rviz_config('path/to/config.rviz')")
    print("rclpy.spin(node)")
    print("node.destroy_node()")
    print("rclpy.shutdown()")
    print("---------------------------\n")
    
    create_launch_file()
    
    print("\nFor full RViz integration, you would need:")
    print("1. A properly configured RViz configuration file")
    print("2. Launch files to start the system")
    print("3. Integration between path planning and visualization")


if __name__ == "__main__":
    main()
