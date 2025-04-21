
import { useState, useEffect } from "react";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";

const Index = () => {
  const [showInfo, setShowInfo] = useState(false);

  useEffect(() => {
    // Welcome message
    console.log("Welcome to the Dijkstra Robot Path Planning project!");
    console.log("This is a React visualization of what would run in Python with ROS2.");
  }, []);

  return (
    <div className="min-h-screen bg-gradient-to-b from-gray-900 to-gray-800 text-white p-8">
      <div className="max-w-4xl mx-auto">
        <header className="text-center mb-12">
          <h1 className="text-5xl font-bold mb-4 text-blue-400">Dijkstra Path Planning Robot</h1>
          <p className="text-xl text-gray-300">
            A ROS2 Humble implementation using Python for optimal path finding
          </p>
        </header>

        <div className="grid grid-cols-1 gap-8 lg:grid-cols-2">
          <Card className="bg-gray-800 border border-gray-700">
            <CardHeader>
              <CardTitle className="text-blue-400">About This Project</CardTitle>
            </CardHeader>
            <CardContent className="text-gray-300">
              <p className="mb-4">
                This project implements a robot that uses Dijkstra's algorithm to find the 
                optimal path to a goal while avoiding obstacles.
              </p>
              <p className="mb-4">
                The implementation uses Python and integrates with ROS2 Humble, RViz for 
                visualization, and Gazebo for simulation.
              </p>
              <Button 
                className="w-full bg-blue-600 hover:bg-blue-700"
                onClick={() => setShowInfo(!showInfo)}
              >
                {showInfo ? "Hide Details" : "Show Implementation Details"}
              </Button>
              
              {showInfo && (
                <div className="mt-4 p-4 bg-gray-700 rounded-md">
                  <h3 className="text-lg font-semibold mb-2 text-blue-300">Key Components:</h3>
                  <ul className="list-disc pl-5 space-y-2">
                    <li>Dijkstra's algorithm implementation for optimal path finding</li>
                    <li>Grid-based map representation with obstacle detection</li>
                    <li>ROS2 integration for real-world robot control</li>
                    <li>Visualization in RViz showing the path planning process</li>
                    <li>Gazebo simulation for testing before real-world deployment</li>
                    <li>Path following controller with smooth motion control</li>
                  </ul>
                </div>
              )}
            </CardContent>
          </Card>

          <Card className="bg-gray-800 border border-gray-700">
            <CardHeader>
              <CardTitle className="text-blue-400">Getting Started</CardTitle>
            </CardHeader>
            <CardContent className="text-gray-300">
              <p className="mb-4">
                To run this project, you need to:
              </p>
              <ol className="list-decimal pl-5 space-y-2 mb-4">
                <li>Install ROS2 Humble on your system</li>
                <li>Clone the project into your ROS2 workspace</li>
                <li>Build the package with colcon</li>
                <li>Source your workspace</li>
                <li>Launch the simulation with the provided launch files</li>
              </ol>
              <div className="p-3 bg-gray-900 rounded font-mono text-sm mb-4">
                <code>
                  $ cd ~/ros2_ws/src<br />
                  $ git clone https://github.com/yourusername/dijkstra_robot<br />
                  $ cd ..<br />
                  $ colcon build --packages-select dijkstra_robot<br />
                  $ source install/setup.bash<br />
                  $ ros2 launch dijkstra_robot robot.launch.py
                </code>
              </div>
              <p>
                Please check the README.md file in the src folder for more detailed instructions.
              </p>
            </CardContent>
          </Card>
        </div>

        <div className="mt-8">
          <Card className="bg-gray-800 border border-gray-700">
            <CardHeader>
              <CardTitle className="text-blue-400">Python Implementation Files</CardTitle>
            </CardHeader>
            <CardContent className="text-gray-300">
              <p className="mb-4">
                This project contains the following Python implementation files:
              </p>
              <ul className="list-disc pl-5 space-y-2">
                <li><strong>dijkstra_robot.py</strong> - Core algorithm implementation with visualization</li>
                <li><strong>ros2_integration.py</strong> - Integration with ROS2 Humble</li>
                <li><strong>gazebo_integration.py</strong> - Integration with Gazebo simulation</li>
                <li><strong>rviz_integration.py</strong> - Visualization in RViz</li>
                <li><strong>robot_controller.py</strong> - Robot motion control for path following</li>
              </ul>
              <p className="mt-4">
                Examine these files to understand the implementation details. The code is extensively
                commented to explain the approach and integration with ROS2 components.
              </p>
            </CardContent>
          </Card>
        </div>
      </div>
    </div>
  );
};

export default Index;
