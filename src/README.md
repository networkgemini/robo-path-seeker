
# Dijkstra Robot Path Planner

This project implements a robot that uses Dijkstra's algorithm to find the fastest path to a goal while avoiding obstacles. It's designed to be integrated with ROS2 Humble, RViz, and Gazebo.

## Files

- `dijkstra_robot.py`: Core implementation of the Dijkstra path planning algorithm with visualization
- `ros2_integration.py`: Template and documentation for integrating with ROS2 Humble

## Requirements

- Python 3.8+
- NumPy
- Matplotlib
- (For ROS2 integration) ROS2 Humble installed

## Running the Simulation

To run the standalone simulation:

```bash
python dijkstra_robot.py
```

This will display a visualization of the Dijkstra algorithm finding a path from the start to the goal, and then simulate the robot moving along that path.

## ROS2 Integration

The `ros2_integration.py` file provides a template for integrating this with ROS2 Humble. To use this in a real ROS2 environment, you would:

1. Create a ROS2 workspace
2. Set up a package structure
3. Modify the integration code as needed
4. Build with colcon
5. Run the node

## How It Works

### The Dijkstra Algorithm

Dijkstra's algorithm finds the shortest path between nodes in a graph. In our case:

1. The grid cells are the nodes
2. Adjacent cells have edges with a weight of 1
3. Obstacles are marked and avoided
4. The algorithm expands from the start node, always choosing the least-cost path first
5. Once the goal is reached, the path is reconstructed

### Integration with ROS2

The ROS2 node:

1. Subscribes to a map topic (`nav_msgs/OccupancyGrid`)
2. Receives goal poses (`geometry_msgs/PoseStamped`)
3. Gets the robot's current position via TF
4. Plans a path using Dijkstra's algorithm
5. Publishes the path for visualization in RViz
6. Controls the robot to follow the path

## Extending the Project

Some ways to extend this project:

1. Add diagonal movements for more natural paths
2. Implement a more sophisticated path following algorithm
3. Add path smoothing
4. Implement dynamic obstacle avoidance
5. Add different cost functions (e.g., prefer paths away from obstacles)
