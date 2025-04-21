
#!/usr/bin/env python3

import heapq
import matplotlib.pyplot as plt
import numpy as np
import time
from matplotlib.colors import ListedColormap
import matplotlib.patches as mpatches

class DijkstraPathPlanner:
    def __init__(self, grid_size=(20, 20)):
        """
        Initialize the Dijkstra Path Planner
        
        Args:
            grid_size: Tuple (width, height) of the grid
        """
        self.grid_size = grid_size
        self.grid = np.zeros(grid_size)  # 0: free, 1: obstacle
        self.start = None
        self.goal = None
        
        # For visualization
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.cmap = ListedColormap(['white', 'black', 'green', 'red', 'blue', 'yellow', 'cyan'])
        self.robot_position = None
        
    def set_start(self, pos):
        """Set start position"""
        self.start = pos
        
    def set_goal(self, pos):
        """Set goal position"""
        self.goal = pos
        
    def add_obstacle(self, pos):
        """Add an obstacle at position"""
        if self._is_valid_pos(pos):
            self.grid[pos] = 1
    
    def add_rectangular_obstacle(self, top_left, width, height):
        """Add a rectangular obstacle"""
        x, y = top_left
        for i in range(x, min(x + width, self.grid_size[0])):
            for j in range(y, min(y + height, self.grid_size[1])):
                self.grid[(i, j)] = 1
                
    def clear_obstacles(self):
        """Clear all obstacles"""
        self.grid = np.zeros(self.grid_size)
        
    def _is_valid_pos(self, pos):
        """Check if position is valid"""
        x, y = pos
        return 0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]
        
    def _get_neighbors(self, pos):
        """Get valid neighbors of a position"""
        x, y = pos
        neighbors = []
        
        # 4-connectivity (up, right, down, left)
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            new_pos = (x + dx, y + dy)
            if self._is_valid_pos(new_pos) and self.grid[new_pos] != 1:
                neighbors.append(new_pos)
                
        return neighbors
    
    def find_path(self, visualize=True):
        """
        Run Dijkstra algorithm to find optimal path
        
        Args:
            visualize: Whether to visualize the algorithm progress
            
        Returns:
            List of positions forming the path or None if no path exists
        """
        if self.start is None or self.goal is None:
            print("Start or goal position not set")
            return None
        
        # Distance dictionary
        dist = {self.start: 0}
        
        # Priority queue for Dijkstra algorithm
        pq = [(0, self.start)]
        
        # Parent dictionary for path reconstruction
        parent = {}
        
        # Visited nodes for visualization
        visited = set()
        
        # Visualization grid
        if visualize:
            vis_grid = self.grid.copy()
            vis_grid[self.start] = 2  # Start (green)
            vis_grid[self.goal] = 3   # Goal (red)
            self._initialize_plot(vis_grid)
        
        # Main Dijkstra algorithm
        while pq:
            current_dist, current_pos = heapq.heappop(pq)
            
            # Skip if we've found a shorter path already
            if current_pos in visited and current_dist > dist[current_pos]:
                continue
                
            # Mark as visited
            visited.add(current_pos)
            
            # If we reached the goal, reconstruct and return the path
            if current_pos == self.goal:
                path = self._reconstruct_path(parent)
                
                if visualize:
                    self._visualize_final_path(vis_grid, path)
                
                return path
            
            # Update visualization
            if visualize and current_pos != self.start:
                vis_grid[current_pos] = 5  # Visited (yellow)
                self._update_plot(vis_grid)
                plt.pause(0.01)  # Small delay for visualization
            
            # Check all neighbors
            for neighbor in self._get_neighbors(current_pos):
                # Calculate new distance (each step costs 1 in this grid)
                new_dist = dist[current_pos] + 1
                
                # If we found a shorter path
                if neighbor not in dist or new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    parent[neighbor] = current_pos
                    heapq.heappush(pq, (new_dist, neighbor))
                    
                    # Update visualization for frontier
                    if visualize and neighbor != self.goal:
                        vis_grid[neighbor] = 6  # Frontier (cyan)
                        self._update_plot(vis_grid)
        
        # If we're here, no path exists
        print("No path found")
        return None
    
    def _reconstruct_path(self, parent):
        """Reconstruct path from parent dictionary"""
        path = []
        current = self.goal
        
        while current != self.start:
            path.append(current)
            current = parent[current]
            
        path.append(self.start)
        path.reverse()
        
        return path
    
    def _initialize_plot(self, vis_grid):
        """Initialize the visualization plot"""
        self.ax.clear()
        self.img = self.ax.imshow(vis_grid.T, cmap=self.cmap, origin='lower')
        self.ax.set_title('Dijkstra Path Planning')
        
        # Create legend
        legend_elements = [
            mpatches.Patch(color='white', label='Free'),
            mpatches.Patch(color='black', label='Obstacle'),
            mpatches.Patch(color='green', label='Start'),
            mpatches.Patch(color='red', label='Goal'),
            mpatches.Patch(color='blue', label='Path'),
            mpatches.Patch(color='yellow', label='Visited'),
            mpatches.Patch(color='cyan', label='Frontier')
        ]
        self.ax.legend(handles=legend_elements, loc='upper right')
        
        plt.draw()
        plt.pause(0.01)
    
    def _update_plot(self, vis_grid):
        """Update the visualization plot"""
        self.img.set_data(vis_grid.T)
        plt.draw()
    
    def _visualize_final_path(self, vis_grid, path):
        """Visualize the final path"""
        for pos in path:
            if pos != self.start and pos != self.goal:
                vis_grid[pos] = 4  # Path (blue)
                self._update_plot(vis_grid)
                plt.pause(0.05)  # Slightly longer pause to see the path
        
        plt.pause(0.5)  # Pause to view the final result
    
    def simulate_robot_movement(self, path):
        """Simulate robot movement along the path"""
        if not path:
            return
            
        vis_grid = self.grid.copy()
        vis_grid[self.start] = 2  # Start (green)
        vis_grid[self.goal] = 3   # Goal (red)
        
        # Mark the path
        for pos in path:
            if pos != self.start and pos != self.goal:
                vis_grid[pos] = 4  # Path (blue)
                
        self._initialize_plot(vis_grid)
        
        # Start robot at first position
        self.robot_position = path[0]
        robot_marker = self.ax.plot(self.robot_position[0], self.robot_position[1], 'ro', markersize=10)[0]
        
        # Move robot along path
        for pos in path[1:]:
            time.sleep(0.5)  # Movement delay
            self.robot_position = pos
            robot_marker.set_data(self.robot_position[0], self.robot_position[1])
            plt.draw()
            plt.pause(0.01)
            
        print("Robot reached the goal!")


# Example usage
if __name__ == "__main__":
    # Create planner with a 20x20 grid
    planner = DijkstraPathPlanner(grid_size=(20, 20))
    
    # Set start and goal
    planner.set_start((2, 2))
    planner.set_goal((18, 18))
    
    # Add some obstacles
    planner.add_rectangular_obstacle((5, 5), 10, 2)
    planner.add_rectangular_obstacle((8, 8), 2, 10)
    
    # Find and visualize path
    path = planner.find_path(visualize=True)
    
    if path:
        print(f"Path found with {len(path)} steps")
        planner.simulate_robot_movement(path)
        
    plt.show()  # Keep plot open
