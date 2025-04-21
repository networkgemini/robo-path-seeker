
import React from 'react';

function App() {
  return (
    <div className="py-10 px-4 min-h-screen flex flex-col items-center justify-center">
      <h1 className="text-3xl font-bold mb-6">ROS2 Robot with Dijkstra Path Planning</h1>
      <p className="text-lg mb-4">This is a Python-based project for ROS2 Humble integration.</p>
      <p className="text-lg mb-4">The main functionality is in the Python files, not in this web interface.</p>
      <div className="mt-6 p-4 bg-gray-100 rounded-lg">
        <h2 className="text-xl font-semibold mb-2">To run the project:</h2>
        <pre className="bg-gray-800 text-white p-4 rounded">python dijkstra_robot.py</pre>
      </div>
    </div>
  );
}

export default App;
