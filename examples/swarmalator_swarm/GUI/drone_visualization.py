#!/usr/bin/env python3
"""
Drone Visualization Module
3D Vispy canvas for real-time drone position visualization
"""

import time
import numpy as np
from typing import Dict

# Vispy imports
import vispy
from vispy import app, scene
from vispy.scene import visuals
from vispy.color import Color
import colorsys

# Local imports
from common import MAX_COPTERS, State

# Configure Vispy to use PyQt6
vispy.use(app='pyqt6')

def angle_to_rgb(angle: float) -> tuple:
    """Convert angle in radians to RGB color"""
    # Normalize angle to [0, 1]
    normalized = (angle % (2 * np.pi)) / (2 * np.pi)
    
    # h, s, v are floats in [0, 1]
    h, s, v = normalized, 1.0, 1.0
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    return Color((r, g, b))

class DroneVisualization:
    """3D Vispy canvas for real-time drone position visualization"""
    
    def __init__(self, parent=None):
        # Create the canvas
        self.canvas = scene.SceneCanvas(keys='interactive', show=True, parent=parent)
        
        # Initialize attributes
        self.drone_markers = []
        self.drone_trails = []
        self.drone_labels = []
        self.position_history = [[] for _ in range(MAX_COPTERS)]
        self.position_timestamps = [[] for _ in range(MAX_COPTERS)]  # Track when each position was added
        self.max_trail_length = 50
        self.trail_decay_time = 10.0  # Trail points fade out after 10 seconds
        
        # Create a viewbox to display the scene
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = 'turntable'
        self.view.camera.fov = 60
        self.view.camera.distance = 6
        
        # Set up the 3D coordinate system
        self._setup_coordinate_system()
        
        # Initialize drone markers
        self._setup_drone_markers()
        
        # Set up timer for continuous trail updates
        self._setup_trail_timer()
    
    @property
    def native(self):
        """Return the native widget for embedding in PyQt"""
        return self.canvas.native
        
    def _setup_coordinate_system(self):
        """Set up 3D coordinate system with axes and grid"""
        # Create coordinate axes
        axis_length = 2.0
        
        # X-axis (red)
        x_axis = scene.visuals.Line(
            pos=np.array([[0, 0, 0], [axis_length, 0, 0]]),
            color='red',
            width=3,
            parent=self.view.scene
        )
        
        # Y-axis (green)
        y_axis = scene.visuals.Line(
            pos=np.array([[0, 0, 0], [0, axis_length, 0]]),
            color='green',
            width=3,
            parent=self.view.scene
        )
        
        # Z-axis (blue)
        z_axis = scene.visuals.Line(
            pos=np.array([[0, 0, 0], [0, 0, axis_length]]),
            color='blue',
            width=3,
            parent=self.view.scene
        )
        
        # Create a ground plane grid
        grid_size = 4
        grid_points = []
        
        # Grid lines parallel to X-axis
        for i in range(-grid_size, grid_size + 1):
            y = i * 0.5
            grid_points.extend([[-grid_size * 0.5, y, 0], [grid_size * 0.5, y, 0]])
            
        # Grid lines parallel to Y-axis
        for i in range(-grid_size, grid_size + 1):
            x = i * 0.5
            grid_points.extend([[x, -grid_size * 0.5, 0], [x, grid_size * 0.5, 0]])
        
        grid = scene.visuals.Line(
            pos=np.array(grid_points),
            color=(0.3, 0.3, 0.3, 0.5),
            width=1,
            parent=self.view.scene,
            connect='segments'
        )
        
    def _setup_drone_markers(self):
        """Initialize visual markers for each drone"""
        colors = [
            'red', 'blue', 'green', 'yellow', 'magenta', 
            'cyan', 'orange', 'purple', 'brown'
        ]
        
        for i in range(MAX_COPTERS):
            # Create sphere marker for drone
            marker = scene.visuals.Sphere(
                radius=0.05,
                color=colors[i % len(colors)],
                parent=self.view.scene
            )
            marker.transform = scene.transforms.MatrixTransform()
            marker.visible = False
            self.drone_markers.append(marker)
            
            # Create trail line for drone path
            trail = scene.visuals.Line(
                color=colors[i % len(colors)],
                width=2,
                parent=self.view.scene
            )
            trail.visible = False
            self.drone_trails.append(trail)
            
            # Create text label for drone ID
            label = scene.visuals.Text(
                text=f'CF{i+1}',
                color='white',
                font_size=12,
                parent=self.view.scene
            )
            label.visible = False
            self.drone_labels.append(label)
    
    def update_drone_position(self, drone_id: int, position: Dict, state: int, phase: float, update_trail: bool = True):
        """Update the 3D position of a specific drone
        
        Args:
            drone_id: ID of the drone to update
            position: Dictionary with x, y, z coordinates
            state: Current state of the drone
            phase: Current phase value (for color)
            update_trail: If False, skip expensive trail visual update (trails will be updated by timer)
        """
        if drone_id < 0 or drone_id >= MAX_COPTERS:
            return
            
        x, y, z = position['x'], position['y'], position['z']
        current_time = time.time()
        
        # Update marker position
        marker = self.drone_markers[drone_id]
        marker.transform.reset()
        marker.transform.translate([x, y, z])
        
        # Show/hide marker based on state
        is_active = state not in [State.STATE_IDLE, State.STATE_UNKNOWN, 255]
        marker.visible = is_active

        # Set drone color to the phase
        marker.mesh.color = angle_to_rgb(phase)
        
        # Update label position
        label = self.drone_labels[drone_id]
        label.pos = (x, y, z + 0.1)  # Slightly above the drone
        label.visible = is_active
        
        # Update trail data (lightweight - just append to list)
        if is_active:
            # Add new position with timestamp
            self.position_history[drone_id].append([x, y, z])
            self.position_timestamps[drone_id].append(current_time)
            
            # Remove old positions based on time and length limits
            self._cleanup_trail(drone_id, current_time)
            
            # Only update trail visual if requested (expensive operation)
            # The trail timer will update visuals periodically anyway
            if update_trail:
                self._update_trail_visual(drone_id, current_time)
        else:
            # Clear trail when drone is inactive
            self.position_history[drone_id].clear()
            self.position_timestamps[drone_id].clear()
            self.drone_trails[drone_id].visible = False
    
    def _cleanup_trail(self, drone_id: int, current_time: float):
        """Remove old trail points based on time and length limits"""
        positions = self.position_history[drone_id]
        timestamps = self.position_timestamps[drone_id]
        
        # Remove points older than decay time
        while timestamps and (current_time - timestamps[0]) > self.trail_decay_time:
            positions.pop(0)
            timestamps.pop(0)
        
        # Also limit by max length
        while len(positions) > self.max_trail_length:
            positions.pop(0)
            timestamps.pop(0)
    
    def _update_trail_visual(self, drone_id: int, current_time: float):
        """Update trail visual with fade effect based on age"""
        positions = self.position_history[drone_id]
        timestamps = self.position_timestamps[drone_id]
        
        if len(positions) < 2:
            self.drone_trails[drone_id].visible = False
            return
        
        # Create trail points array
        trail_points = np.array(positions)
        
        # Calculate alpha values based on age (newer = more opaque)
        alphas = []
        for timestamp in timestamps:
            age = current_time - timestamp
            # Linear fade from 1.0 (newest) to 0.1 (oldest)
            alpha = max(0.1, 1.0 - (age / self.trail_decay_time))
            alphas.append(alpha)
        
        # Get base color for this drone
        colors = [
            'red', 'blue', 'green', 'yellow', 'magenta', 
            'cyan', 'orange', 'purple', 'brown'
        ]
        base_color = Color(colors[drone_id % len(colors)])
        
        # Get RGBA components from the color (returns array [r, g, b, a])
        rgba = base_color.rgba
        
        # Create color array with varying alpha
        trail_colors = []
        for alpha in alphas:
            color_with_alpha = [rgba[0], rgba[1], rgba[2], alpha]
            trail_colors.append(color_with_alpha)
        
        # Update the trail
        trail = self.drone_trails[drone_id]
        trail.set_data(trail_points, color=np.array(trail_colors))
        trail.visible = True
    
    def _setup_trail_timer(self):
        """Set up timer for continuous trail decay updates"""
        from vispy import app
        self.trail_timer = app.Timer('auto', connect=self._update_all_trails, start=True)
        self.trail_timer.interval = 0.1  # Update every 100ms for smooth fading
    
    def _update_all_trails(self, event):
        """Update all trails for continuous decay effect"""
        current_time = time.time()
        
        for drone_id in range(MAX_COPTERS):
            if self.position_history[drone_id]:  # Only update if drone has trail data
                # Clean up old points
                self._cleanup_trail(drone_id, current_time)
                
                # Update visual with current fade
                self._update_trail_visual(drone_id, current_time)
    
    def set_trail_decay_time(self, decay_time: float):
        """Set the trail decay time in seconds"""
        self.trail_decay_time = decay_time
    
    def set_max_trail_length(self, max_length: int):
        """Set the maximum trail length in points"""
        self.max_trail_length = max_length