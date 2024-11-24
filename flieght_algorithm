from dataclasses import dataclass
from typing import Tuple, List, Set
import math
import random
import time
import threading
import numpy as np
from queue import Queue
import tkinter as tk
from tkintermapview import TkinterMapView
from scipy.interpolate import interp1d

@dataclass
class MapCell:
    known: bool = False
    has_fire: bool = False
    last_checked: float = 0
    cloud_coverage: float = 1.0
    inspected: bool = False  # New field to track if cell was inspected

class Cloud:
    def __init__(self, center: Tuple[float, float], radius: float):
        self.center = center
        self.radius = radius
        self.opacity = random.uniform(0.3, 0.8)
        self.inspection_points: Set[Tuple[int, int]] = set()  # Points that need inspection

class DronePathPlanner:
    def __init__(self):
        self.current_position = (48.2297, 20.0122)
        self.target_position = None
        self.current_path = []
        self.location_queue = Queue()
        self.target_marker = None
        self.flight_mode = "idle"
        self.path_line = None
        self.inspection_markers = []

        self.map_size = (100, 100)
        self.cell_size = 0.001
        self.map_data = np.array([[MapCell() for _ in range(self.map_size[1])] 
                                  for _ in range(self.map_size[0])])
        self.clouds = []
        self.cloud_polygons = []
        self.show_clouds = True
        self.drone_speed = 0.0001
        self.scan_radius = 3
        self.inspection_interval = 2.0
        self.last_inspection_time = 0

        self.setup_gui()
        self.generate_random_clouds(5)

    def generate_random_clouds(self, count: int):
        """Generate random clouds and calculate inspection points."""
        self.clouds.clear()
        center_lat, center_lon = self.current_position
        
        for _ in range(count):
            cloud_lat = center_lat + random.uniform(-0.01, 0.01)
            cloud_lon = center_lon + random.uniform(-0.01, 0.01)
            cloud_radius = random.uniform(0.002, 0.005)
            cloud = Cloud((cloud_lat, cloud_lon), cloud_radius)
            
            # Calculate inspection points for the cloud
            cloud_grid_pos = self.get_grid_coordinates(*cloud.center)
            cloud_grid_radius = int(cloud.radius / self.cell_size)
            
            # Generate inspection points in a grid pattern within the cloud
            inspection_spacing = self.scan_radius * 2  # Space between inspection points
            for dx in range(-cloud_grid_radius, cloud_grid_radius + 1, inspection_spacing):
                for dy in range(-cloud_grid_radius, cloud_grid_radius + 1, inspection_spacing):
                    x, y = cloud_grid_pos[0] + dx, cloud_grid_pos[1] + dy
                    if (0 <= x < self.map_size[0] and 0 <= y < self.map_size[1]):
                        distance = math.sqrt(dx**2 + dy**2)
                        if distance <= cloud_grid_radius:
                            cloud.inspection_points.add((x, y))
            
            self.clouds.append(cloud)
            self.update_cloud_coverage(cloud)
        
        self.update_cloud_visualization()
        self.visualize_inspection_points()

    def visualize_inspection_points(self):
        """Visualize inspection points on the map."""
        # Clear existing inspection markers
        for marker in self.inspection_markers:
            self.map_widget.delete(marker)
        self.inspection_markers.clear()

        # Add new inspection markers
        for cloud in self.clouds:
            for grid_x, grid_y in cloud.inspection_points:
                lat, lon = self.get_geographic_coordinates(grid_x, grid_y)
                marker = self.map_widget.set_marker(
                    lat, lon,
                    text="•",
                    text_color="yellow",
                    marker_color_circle="gray",
                    marker_color_outside="gray"
                )
                self.inspection_markers.append(marker)

    def update_cloud_visualization(self):
        """Update cloud visualization on map."""
        # Clear existing cloud polygons
        for polygon in self.cloud_polygons:
            self.map_widget.delete(polygon)
        self.cloud_polygons.clear()
        
        if not self.show_clouds:
            return
        
        # Generate new cloud polygons
        for cloud in self.clouds:
            # Generate circle points for cloud
            points = []
            steps = 32
            for i in range(steps):
                angle = (2 * math.pi * i) / steps
                lat = cloud.center[0] + math.sin(angle) * cloud.radius
                lon = cloud.center[1] + math.cos(angle) * cloud.radius
                points.append((lat, lon))
            
            # Convert opacity to hex color
            opacity_value = int(128 + (cloud.opacity * 127))
            color = f"#{opacity_value:02x}{opacity_value:02x}{opacity_value:02x}"
            
            # Create cloud polygon
            polygon = self.map_widget.set_polygon(
                points,
                fill_color=color,
                outline_color="#ffffff",
                border_width=1,
                name=f"cloud_{id(cloud)}"
            )
            self.cloud_polygons.append(polygon)

    def on_map_click(self, coords):
        """Handle map click and update path."""
        lat, lon = coords
        self.target_position = coords
        
        # Update or create target marker
        if self.target_marker:
            self.target_marker.set_position(lat, lon)
        else:
            self.target_marker = self.map_widget.set_marker(
                lat, lon,
                text="Target",
                marker_color_circle="red"
            )
        
        # Calculate path
        start = self.get_grid_coordinates(*self.current_position)
        target = self.get_grid_coordinates(lat, lon)
        grid_path = self.find_path(start, target)
        self.current_path = [self.get_geographic_coordinates(x, y) for x, y in grid_path]
        
        # Update path visualization
        self.update_path_display()
        
        # Start movement if not already running
        if not hasattr(self, 'movement_thread') or not self.movement_thread.is_alive():
            self.movement_thread = threading.Thread(target=self.move_drone)
            self.movement_thread.daemon = True
            self.movement_thread.start()

    def update_path_display(self):
        """Update path visualization on map."""
        # Remove existing path
        if self.path_line:
            self.map_widget.delete(self.path_line)
        
        # Draw new path if exists
        if self.current_path:
            self.path_line = self.map_widget.set_path(
                self.current_path,
                color="blue",
                width=2
            )
   
    def move_drone(self):
        """Move drone along a smooth curved path with cloud inspection."""
        if len(self.current_path) < 2:
            return  # Not enough points for a path
        
        # Generate smooth path using Bezier-like interpolation
        def generate_smooth_path(path_points, num_interpolated_points=100):
            path_points = np.array(path_points)
            latitudes = path_points[:, 0]
            longitudes = path_points[:, 1]
            
            # Generate smooth interpolation
            t = np.linspace(0, 1, len(path_points))  # Parameter t for interpolation
            interp_lat = interp1d(t, latitudes, kind='cubic')  # Cubic interpolation for latitudes
            interp_lon = interp1d(t, longitudes, kind='cubic')  # Cubic interpolation for longitudes
            
            # Create fine-grained interpolation points
            t_fine = np.linspace(0, 1, num_interpolated_points)
            smooth_latitudes = interp_lat(t_fine)
            smooth_longitudes = interp_lon(t_fine)
            return list(zip(smooth_latitudes, smooth_longitudes))
        
        # Generate smooth path
        smooth_path = generate_smooth_path(self.current_path)
        
        # Drone movement loop
        for next_position in smooth_path:
            current_time = time.time()
            current_lat, current_lon = self.current_position
            current_grid = self.get_grid_coordinates(current_lat, current_lon)
            
            # Check if it's time for inspection
            if current_time - self.last_inspection_time >= self.inspection_interval:
                self.inspect_area(current_grid)
                self.last_inspection_time = current_time

            # Move drone to next position
            dlat = next_position[0] - current_lat
            dlon = next_position[1] - current_lon
            distance = math.sqrt(dlat**2 + dlon**2)
            
            if distance < self.drone_speed:
                self.current_position = next_position
            else:
                ratio = self.drone_speed / distance
                new_lat = current_lat + dlat * ratio
                new_lon = current_lon + dlon * ratio
                self.current_position = (new_lat, new_lon)
            
            # Update drone marker and sleep for smooth movement
            self.location_queue.put(self.current_position)
            time.sleep(0.05)

        self.flight_mode = "idle"  # Mark the flight as complete

    def inspect_area(self, current_grid: Tuple[int, int]):
        """Inspect the area around the drone's current position."""
        x, y = current_grid
        for dx in range(-self.scan_radius, self.scan_radius + 1):
            for dy in range(-self.scan_radius, self.scan_radius + 1):
                inspect_x, inspect_y = x + dx, y + dy
                if (0 <= inspect_x < self.map_size[0] and 
                    0 <= inspect_y < self.map_size[1]):
                    distance = math.sqrt(dx**2 + dy**2)
                    if distance <= self.scan_radius:
                        # Mark cell as inspected
                        self.map_data[inspect_x][inspect_y].inspected = True
                        self.map_data[inspect_x][inspect_y].last_checked = time.time()
                        
                        # Remove inspection points that have been checked
                        for cloud in self.clouds:
                            if (inspect_x, inspect_y) in cloud.inspection_points:
                                cloud.inspection_points.remove((inspect_x, inspect_y))
        
        # Update visualization
        self.visualize_inspection_points()

    def find_path(self, start: Tuple[int, int], target: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Modified A* pathfinding that considers cloud inspection points."""
        def heuristic(a, b):
            return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

        def get_inspection_value(pos):
            """Calculate the value of a position based on nearby inspection points."""
            value = 0
            for cloud in self.clouds:
                for inspect_point in cloud.inspection_points:
                    dist = math.sqrt((pos[0] - inspect_point[0])**2 + (pos[1] - inspect_point[1])**2)
                    if dist <= self.scan_radius:
                        value += 1 / (dist + 1)  # Higher value for closer inspection points
            return value

        frontier = [(0, start)]
        came_from = {start: None}
        cost_so_far = {start: 0}
        inspection_bonus = 0.3  # Weight for inspection value in path planning

        while frontier:
            current = frontier.pop(0)[1]
            if current == target:
                break

            for dx, dy in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]:
                next_cell = (current[0] + dx, current[1] + dy)
                if (0 <= next_cell[0] < self.map_size[0] and 
                    0 <= next_cell[1] < self.map_size[1]):
                    
                    # Base movement cost
                    movement_cost = math.sqrt(dx**2 + dy**2)
                    
                    # Add inspection value influence
                    inspection_value = get_inspection_value(next_cell)
                    adjusted_cost = movement_cost - (inspection_value * inspection_bonus)
                    
                    new_cost = cost_so_far[current] + adjusted_cost
                    
                    if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                        cost_so_far[next_cell] = new_cost
                        priority = new_cost + heuristic(next_cell, target)
                        # Insert maintaining sorted order
                        i = 0
                        while i < len(frontier) and frontier[i][0] < priority:
                            i += 1
                        frontier.insert(i, (priority, next_cell))
                        came_from[next_cell] = current

        path = []
        current = target
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()
        return path

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.geometry("1000x800")
        self.root.title("Smart Drone Path Planner")

        self.control_panel = tk.Frame(self.root)
        self.control_panel.pack(side="top", fill="x", padx=5, pady=5)

        speed_frame = tk.Frame(self.control_panel)
        speed_frame.pack(side="left", padx=5)
        tk.Label(speed_frame, text="Drone Speed:").pack(side="left")
        self.speed_scale = tk.Scale(
            speed_frame,
            from_=0.00001,
            to=0.001,
            resolution=0.00001,
            orient="horizontal",
            command=self.update_drone_speed
        )
        self.speed_scale.set(self.drone_speed)
        self.speed_scale.pack(side="left")

        cloud_frame = tk.Frame(self.control_panel)
        cloud_frame.pack(side="left", padx=5)
        self.cloud_var = tk.BooleanVar(value=True)
        self.cloud_toggle = tk.Checkbutton(
            cloud_frame,
            text="Show Clouds",
            variable=self.cloud_var,
            command=self.toggle_clouds
        )
        self.cloud_toggle.pack(side="left")

        self.generate_clouds_button = tk.Button(
            cloud_frame,
            text="Generate New Clouds",
            command=lambda: self.generate_random_clouds(5)
        )
        self.generate_clouds_button.pack(side="left", padx=5)

        self.map_widget = TkinterMapView(self.root, width=1000, height=700, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        self.map_widget.set_position(*self.current_position)
        self.map_widget.set_zoom(15)

        self.drone_marker = self.map_widget.set_marker(
            *self.current_position,
            text="Drone",
            marker_color_circle="blue",
            marker_color_outside="gray"
        )

        # Bind click event
        self.map_widget.add_left_click_map_command(self.on_map_click)

    def update_cloud_visualization(self):
        """Update cloud visualization on map."""
        # Clear existing cloud polygons
        for polygon in self.cloud_polygons:
            self.map_widget.delete(polygon)
        self.cloud_polygons.clear()
        
        if not self.show_clouds:
            return
        
        # Generate new cloud polygons
        for cloud in self.clouds:
            # Generate circle points for cloud
            points = []
            steps = 32
            for i in range(steps):
                angle = (2 * math.pi * i) / steps
                lat = cloud.center[0] + math.sin(angle) * cloud.radius
                lon = cloud.center[1] + math.cos(angle) * cloud.radius
                points.append((lat, lon))
            
            # Convert opacity to hex color
            opacity_value = int(128 + (cloud.opacity * 127))
            color = f"#{opacity_value:02x}{opacity_value:02x}{opacity_value:02x}"
            
            # Create cloud polygon
            polygon = self.map_widget.set_polygon(
                points,
                fill_color=color,
                outline_color="#ffffff",
                border_width=1,
                name=f"cloud_{id(cloud)}"
            )
            self.cloud_polygons.append(polygon)

    def on_map_click(self, coords):
        """Handle map click and update path."""
        lat, lon = coords
        self.target_position = coords
        
        # Update or create target marker
        if self.target_marker:
            self.target_marker.set_position(lat, lon)
        else:
            self.target_marker = self.map_widget.set_marker(
                lat, lon,
                text="Target",
                marker_color_circle="red"
            )
        
        # Calculate path
        start = self.get_grid_coordinates(*self.current_position)
        target = self.get_grid_coordinates(lat, lon)
        grid_path = self.find_path(start, target)
        self.current_path = [self.get_geographic_coordinates(x, y) for x, y in grid_path]
        
        # Update path visualization
        self.update_path_display()
        
        # Start movement if not already running
        if not hasattr(self, 'movement_thread') or not self.movement_thread.is_alive():
            self.movement_thread = threading.Thread(target=self.move_drone)
            self.movement_thread.daemon = True
            self.movement_thread.start()

    def update_path_display(self):
        """Update path visualization on map."""
        # Remove existing path
        if hasattr(self, 'path_line') and self.path_line:
            self.map_widget.delete(self.path_line)
        
        # Draw new path if exists
        if self.current_path:
            self.path_line = self.map_widget.set_path(
                self.current_path,
                color="blue",
                width=2
            )

    def get_grid_coordinates(self, lat: float, lon: float) -> Tuple[int, int]:
        center_lat, center_lon = self.current_position
        x = int((lon - center_lon) / self.cell_size + self.map_size[0] / 2)
        y = int((lat - center_lat) / self.cell_size + self.map_size[1] / 2)
        return max(0, min(x, self.map_size[0] - 1)), max(0, min(y, self.map_size[1] - 1))

    def get_geographic_coordinates(self, x: int, y: int) -> Tuple[float, float]:
        center_lat, center_lon = self.current_position
        lon = (x - self.map_size[0] / 2) * self.cell_size + center_lon
        lat = (y - self.map_size[1] / 2) * self.cell_size + center_lat
        return lat, lon

    def update_cloud_coverage(self, cloud: Cloud):
        cloud_grid_pos = self.get_grid_coordinates(*cloud.center)
        cloud_grid_radius = int(cloud.radius / self.cell_size)

        for dx in range(-cloud_grid_radius, cloud_grid_radius + 1):
            for dy in range(-cloud_grid_radius, cloud_grid_radius + 1):
                x, y = cloud_grid_pos[0] + dx, cloud_grid_pos[1] + dy
                if (0 <= x < self.map_size[0] and 0 <= y < self.map_size[1]):
                    distance = math.sqrt(dx**2 + dy**2)
                    if distance <= cloud_grid_radius:
                        coverage = (1 - (distance / cloud_grid_radius)) * cloud.opacity
                        current_coverage = self.map_data[x][y].cloud_coverage
                        self.map_data[x][y].cloud_coverage = min(1, current_coverage + coverage)

    def update_drone_speed(self, value):
        self.drone_speed = float(value)

    def toggle_clouds(self):
        self.show_clouds = self.cloud_var.get()
        self.update_cloud_visualization()

    def update_display(self):
        """Update display elements."""
        try:
            while not self.location_queue.empty():
                position = self.location_queue.get_nowait()
                self.drone_marker.set_position(*position)
        except Queue.Empty:
            pass
            
        self.root.after(50, self.update_display)

    def run(self):
        """Start the application."""
        self.root.after(50, self.update_display)
        self.root.mainloop()

if __name__ == "__main__":
    planner = DronePathPlanner()
    planner.run()
