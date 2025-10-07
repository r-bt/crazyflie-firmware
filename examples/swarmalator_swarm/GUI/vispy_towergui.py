#!/usr/bin/env python3
"""
PyQt + Vispy GUI for Crazyflie Swarm Control Tower
Combines traditional GUI controls with 3D real-time drone visualization
"""

import sys
import time
import threading
import zmq
import numpy as np
import json
import os
from datetime import datetime
from queue import Queue
from typing import List, Dict, Optional
from colorama import Fore

# PyQt imports
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QLabel, QPushButton, QProgressBar, QCheckBox,
    QSplitter, QFrame, QMessageBox, QGroupBox, QSlider,
    QFileDialog, QButtonGroup
)
from PyQt6.QtCore import QTimer, pyqtSignal, QObject, Qt
from PyQt6.QtGui import QFont, QPalette, QColor

# Vispy imports
import vispy
from vispy import app, scene
from vispy.scene import visuals
from vispy.color import Color

# Local imports
from common import MAX_COPTERS, state_dict, State
from sniffer_interface import snifferThread

# Configure Vispy to use PyQt6
vispy.use(app='pyqt6')

COPTER_ALIVE_TIMEOUT = 2  # sec


class ExperimentRecorder:
    """Handles recording and replaying of experiment data"""
    
    def __init__(self):
        self.is_recording = False
        self.is_replaying = False
        self.recorded_data = []
        self.replay_data = []
        self.replay_start_time = None
        self.replay_index = 0
        self.experiment_start_time = None
        
    def start_recording(self):
        """Start recording experiment data"""
        self.is_recording = True
        self.recorded_data = []
        self.experiment_start_time = time.time()
        print("Started recording experiment data...")
        
    def stop_recording(self):
        """Stop recording experiment data"""
        self.is_recording = False
        print(f"Stopped recording. Captured {len(self.recorded_data)} data points.")
        
    def record_drone_data(self, drone_id: int, data: Dict):
        """Record a single drone data point"""
        if not self.is_recording:
            return
            
        timestamp = time.time() - self.experiment_start_time
        
        record = {
            'timestamp': timestamp,
            'drone_id': drone_id,
            'position': data['position'].copy(),
            'state': data['state'],
            'battery': data['battery'],
            'counter': data.get('counter', 0),
            'phase': data.get('phase', 0)
        }
        
        self.recorded_data.append(record)
        
    def save_to_file(self, filename: str) -> bool:
        """Save recorded data to JSON file"""
        try:
            experiment_data = {
                'metadata': {
                    'recorded_at': datetime.now().isoformat(),
                    'duration': self.recorded_data[-1]['timestamp'] if self.recorded_data else 0,
                    'total_points': len(self.recorded_data),
                    'drones_involved': list(set(record['drone_id'] for record in self.recorded_data))
                },
                'data': self.recorded_data
            }
            
            with open(filename, 'w') as f:
                json.dump(experiment_data, f, indent=2)
                
            print(f"Experiment data saved to {filename}")
            return True
            
        except Exception as e:
            print(f"Error saving experiment data: {e}")
            return False
            
    def load_from_file(self, filename: str) -> bool:
        """Load experiment data from JSON file"""
        try:
            with open(filename, 'r') as f:
                experiment_data = json.load(f)
                
            self.replay_data = experiment_data['data']
            self.replay_index = 0
            
            print(f"Loaded experiment data from {filename}")
            print(f"Duration: {experiment_data['metadata']['duration']:.1f}s")
            print(f"Total points: {experiment_data['metadata']['total_points']}")
            print(f"Drones involved: {experiment_data['metadata']['drones_involved']}")
            
            return True
            
        except Exception as e:
            print(f"Error loading experiment data: {e}")
            return False
            
    def start_replay(self):
        """Start replaying loaded experiment data"""
        if not self.replay_data:
            return False
            
        self.is_replaying = True
        self.replay_start_time = time.time()
        self.replay_index = 0
        print("Started replaying experiment...")
        return True
        
    def stop_replay(self):
        """Stop replaying experiment data"""
        self.is_replaying = False
        print("Stopped replaying experiment.")
        
    def get_replay_data(self) -> List[Dict]:
        """Get current replay data points based on elapsed time"""
        if not self.is_replaying or not self.replay_data:
            return []
            
        current_replay_time = time.time() - self.replay_start_time
        replay_points = []
        
        # Find all data points that should be shown at current time
        while (self.replay_index < len(self.replay_data) and 
               self.replay_data[self.replay_index]['timestamp'] <= current_replay_time):
            replay_points.append(self.replay_data[self.replay_index])
            self.replay_index += 1
            
        return replay_points
        
    def is_replay_finished(self) -> bool:
        """Check if replay has finished"""
        return self.is_replaying and self.replay_index >= len(self.replay_data)


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
    
    def update_drone_position(self, drone_id: int, position: Dict, state: int):
        """Update the 3D position of a specific drone"""
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
        
        # Update label position
        label = self.drone_labels[drone_id]
        label.pos = (x, y, z + 0.1)  # Slightly above the drone
        label.visible = is_active
        
        # Update trail with time-based decay
        if is_active:
            # Add new position with timestamp
            self.position_history[drone_id].append([x, y, z])
            self.position_timestamps[drone_id].append(current_time)
            
            # Remove old positions based on time and length limits
            self._cleanup_trail(drone_id, current_time)
            
            # Update trail visual with fade effect
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


class CrazyflieWidget(QGroupBox):
    """PyQt widget for displaying individual Crazyflie status"""
    
    def __init__(self, drone_id: int, parent=None):
        super().__init__(f"Crazyflie #{drone_id}", parent)
        self.drone_id = drone_id
        self._setup_ui()
        
        # Initialize state
        self.prev_counter = None
        self.position = {"x": 0, "y": 0, "z": 0}
        
    def _setup_ui(self):
        """Set up the UI components"""
        layout = QVBoxLayout()
        
        # Status LED indicator
        self.led_label = QLabel("●")
        self.led_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.led_label.setStyleSheet("color: grey; font-size: 20px;")
        layout.addWidget(self.led_label)
        
        # Status text
        self.status_label = QLabel("IDLE")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        font = QFont()
        font.setPointSize(16)
        font.setBold(True)
        self.status_label.setFont(font)
        self.status_label.setStyleSheet("color: grey;")
        layout.addWidget(self.status_label)
        
        # Battery section
        battery_layout = QHBoxLayout()
        battery_layout.addWidget(QLabel("Battery:"))
        
        self.battery_voltage_label = QLabel("0.00V")
        battery_layout.addWidget(self.battery_voltage_label)
        
        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(0)
        battery_layout.addWidget(self.battery_bar)
        
        layout.addLayout(battery_layout)
        
        # Position info
        self.position_label = QLabel("Pos: (0.00, 0.00, 0.00)")
        self.position_label.setStyleSheet("font-family: monospace; font-size: 10px;")
        layout.addWidget(self.position_label)
        
        self.setLayout(layout)
        
    def set_led(self, color: str):
        """Set LED indicator color"""
        self.led_label.setStyleSheet(f"color: {color}; font-size: 20px;")
        # Reset to grey after 1 second
        QTimer.singleShot(1000, lambda: self.led_label.setStyleSheet("color: grey; font-size: 20px;"))
        
    def set_state(self, state: int):
        """Update drone state display"""
        if state in state_dict:
            text, color = state_dict[state]
            self.status_label.setText(text)
            self.status_label.setStyleSheet(f"color: {color}; font-weight: bold;")
        else:
            self.status_label.setText("ERROR")
            self.status_label.setStyleSheet("color: grey; font-weight: bold;")
            
    def set_battery(self, voltage: float):
        """Update battery display"""
        self.battery_voltage_label.setText(f"{voltage:.2f}V")
        
        # Calculate percentage (3.0V = 0%, 4.1V = 100%)
        percent = max(0, min(100, (voltage - 3.0) * 100.0 / 1.1))
        self.battery_bar.setValue(int(percent))
        
        # Color code the battery bar
        if percent > 50:
            color = "green"
        elif percent > 20:
            color = "orange"
        else:
            color = "red"
            
        self.battery_bar.setStyleSheet(f"""
            QProgressBar::chunk {{
                background-color: {color};
            }}
        """)
        
    def set_position(self, position: Dict):
        """Update position display"""
        self.position = position
        x, y, z = position['x'], position['y'], position['z']
        self.position_label.setText(f"Pos: ({x:.2f}, {y:.2f}, {z:.2f})")
        
    def is_updated(self, counter: int) -> bool:
        """Check if drone data has been updated"""
        if self.prev_counter is None:
            self.prev_counter = counter
            return True
            
        if counter != self.prev_counter:
            self.prev_counter = counter
            return True
        else:
            return False


class DataUpdateSignals(QObject):
    """Qt signals for thread-safe GUI updates"""
    drone_updated = pyqtSignal(int, dict)  # drone_id, data
    error_occurred = pyqtSignal(str, str)  # title, message


class SwarmControlTower(QMainWindow):
    """Main application window"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Decentralized Swarm Control Tower - 3D Visualization")
        self.setGeometry(100, 100, 1400, 800)
        
        # Initialize components
        self.drone_widgets: List[CrazyflieWidget] = []
        self.signals = DataUpdateSignals()
        self.signals.drone_updated.connect(self.update_drone_display)
        self.signals.error_occurred.connect(self.show_error)
        
        # Initialize experiment recorder
        self.recorder = ExperimentRecorder()
        self.experiment_running = False
        
        # ZMQ setup
        self._setup_zmq()
        
        # GUI setup
        self._setup_ui()
        
        # Start background threads
        self._start_threads()
        
        # Set up replay timer
        self.replay_timer = QTimer()
        self.replay_timer.timeout.connect(self._update_replay)
        self.replay_timer.setInterval(50)  # Update every 50ms for smooth replay
        
    def _setup_zmq(self):
        """Initialize ZMQ sockets"""
        self.context = zmq.Context()
        
        self.report_socket = self.context.socket(zmq.PULL)
        self.report_socket.connect("tcp://127.0.0.1:5555")
        self.report_socket.setsockopt(zmq.RCVTIMEO, 1000)
        
        self.command_socket = self.context.socket(zmq.PUSH)
        try:
            self.command_socket.bind("tcp://*:5556")
        except zmq.error.ZMQError as e:
            if "Address already in use" in str(e):
                print("Warning: Port 5556 already in use. Trying alternative port 5557...")
                try:
                    self.command_socket.bind("tcp://*:5557")
                    print("Successfully bound to port 5557")
                except zmq.error.ZMQError:
                    print("Error: Could not bind to any port. Please close other instances.")
                    raise
            else:
                raise
        
    def _setup_ui(self):
        """Set up the main UI"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout with splitter
        main_layout = QHBoxLayout()
        splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # Left panel: Drone status widgets
        left_panel = self._create_drone_panel()
        splitter.addWidget(left_panel)
        
        # Right panel: 3D visualization
        right_panel = self._create_visualization_panel()
        splitter.addWidget(right_panel)
        
        # Set splitter proportions (30% left, 70% right)
        splitter.setSizes([400, 1000])
        
        main_layout.addWidget(splitter)
        central_widget.setLayout(main_layout)
        
    def _create_drone_panel(self) -> QWidget:
        """Create the left panel with drone status widgets"""
        panel = QWidget()
        layout = QVBoxLayout()
        
        # Control buttons
        controls_group = QGroupBox("Experiment Controls")
        controls_layout = QVBoxLayout()
        
        self.toggle_button = QCheckBox("Experiment Running")
        self.toggle_button.clicked.connect(self.toggle_experiment)
        controls_layout.addWidget(self.toggle_button)
        
        # Recording controls
        recording_layout = QHBoxLayout()
        
        self.save_button = QPushButton("Save Experiment")
        self.save_button.clicked.connect(self.save_experiment)
        self.save_button.setEnabled(False)
        recording_layout.addWidget(self.save_button)
        
        self.load_button = QPushButton("Load Experiment")
        self.load_button.clicked.connect(self.load_experiment)
        recording_layout.addWidget(self.load_button)
        
        controls_layout.addLayout(recording_layout)
        
        # Replay controls
        replay_layout = QHBoxLayout()
        
        self.replay_button = QPushButton("Start Replay")
        self.replay_button.clicked.connect(self.toggle_replay)
        self.replay_button.setEnabled(False)
        replay_layout.addWidget(self.replay_button)
        
        self.stop_replay_button = QPushButton("Stop Replay")
        self.stop_replay_button.clicked.connect(self.stop_replay)
        self.stop_replay_button.setEnabled(False)
        replay_layout.addWidget(self.stop_replay_button)
        
        controls_layout.addLayout(replay_layout)
        
        # Status label for recording/replay
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("color: blue; font-weight: bold;")
        controls_layout.addWidget(self.status_label)
        
        # Trail controls
        trail_group = QGroupBox("Trail Settings")
        trail_layout = QVBoxLayout()
        
        # Decay time slider
        decay_layout = QHBoxLayout()
        decay_layout.addWidget(QLabel("Decay Time:"))
        self.decay_slider = QSlider(Qt.Orientation.Horizontal)
        self.decay_slider.setRange(1, 30)  # 1-30 seconds
        self.decay_slider.setValue(10)  # Default 10 seconds
        self.decay_slider.valueChanged.connect(self.update_decay_time)
        decay_layout.addWidget(self.decay_slider)
        self.decay_label = QLabel("10s")
        decay_layout.addWidget(self.decay_label)
        trail_layout.addLayout(decay_layout)
        
        # Trail length slider
        length_layout = QHBoxLayout()
        length_layout.addWidget(QLabel("Trail Length:"))
        self.length_slider = QSlider(Qt.Orientation.Horizontal)
        self.length_slider.setRange(10, 100)  # 10-100 points
        self.length_slider.setValue(50)  # Default 50 points
        self.length_slider.valueChanged.connect(self.update_trail_length)
        length_layout.addWidget(self.length_slider)
        self.length_label = QLabel("50")
        length_layout.addWidget(self.length_label)
        trail_layout.addLayout(length_layout)
        
        trail_group.setLayout(trail_layout)
        controls_layout.addWidget(trail_group)
        
        controls_group.setLayout(controls_layout)
        layout.addWidget(controls_group)
        
        # Drone status widgets in a grid
        drones_group = QGroupBox("Drone Status")
        drones_layout = QGridLayout()
        
        for i in range(MAX_COPTERS):
            drone_widget = CrazyflieWidget(i + 1)
            row = i // 3
            col = i % 3
            drones_layout.addWidget(drone_widget, row, col)
            self.drone_widgets.append(drone_widget)
            
            # Initialize with error state
            drone_widget.set_state(State.STATE_UNKNOWN)
            drone_widget.set_battery(0.0)
            
        drones_group.setLayout(drones_layout)
        layout.addWidget(drones_group)
        
        layout.addStretch()  # Push everything to the top
        panel.setLayout(layout)
        return panel
        
    def _create_visualization_panel(self) -> QWidget:
        """Create the right panel with 3D visualization"""
        panel = QWidget()
        layout = QVBoxLayout()
        
        # Title
        title = QLabel("Real-time 3D Drone Positions")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        font = QFont()
        font.setPointSize(14)
        font.setBold(True)
        title.setFont(font)
        layout.addWidget(title)
        
        # 3D visualization canvas
        self.visualization = DroneVisualization()
        layout.addWidget(self.visualization.native)
        
        # Instructions
        instructions = QLabel(
            "Controls: Left click + drag to rotate • Right click + drag to zoom • Middle click + drag to pan"
        )
        instructions.setAlignment(Qt.AlignmentFlag.AlignCenter)
        instructions.setStyleSheet("color: grey; font-size: 10px;")
        layout.addWidget(instructions)
        
        panel.setLayout(layout)
        return panel
        
    def _start_threads(self):
        """Start background threads"""
        # Start sniffer thread
        self.sniffer_thread = snifferThread()
        self.sniffer_thread.start()
        
        # Start data receiving thread
        self.receive_thread = threading.Thread(target=self._receive_data, daemon=True)
        self.receive_thread.start()
        
    def _receive_data(self):
        """Background thread for receiving ZMQ data"""
        last_updated = [0] * MAX_COPTERS
        
        while True:
            try:
                report = self.report_socket.recv_json()
                
                # If we're replaying a recording, ignore any new live measurements to avoid conflicts
                if hasattr(self, 'recorder') and self.recorder.is_replaying:
                    continue

                if report[0] == "connection_failed":
                    self.signals.error_occurred.emit(
                        "Connection Error",
                        "Connection with sniffer failed! Please try again."
                    )
                    break
                    
                # Process drone data
                for data in report:
                    if data.get("id") != "action" and isinstance(data.get("id"), int):
                        drone_id = data["id"] - 1  # Convert to 0-based index
                        
                        if 0 <= drone_id < MAX_COPTERS:
                            # Emit signal for GUI update
                            self.signals.drone_updated.emit(drone_id, data)
                            
                            # Check for updates
                            if self.drone_widgets[drone_id].is_updated(data["counter"]):
                                self.drone_widgets[drone_id].set_led("green")
                                last_updated[drone_id] = time.time()
                                
            except zmq.error.Again:
                pass  # Timeout, continue
            except Exception as e:
                print(f"Error in receive thread: {e}")
                
            # Check for timeouts
            current_time = time.time()
            for i in range(MAX_COPTERS):
                if (current_time - last_updated[i] > COPTER_ALIVE_TIMEOUT and 
                    i != 8):  # Skip drone 9 (index 8) as noted in original code
                    
                    timeout_data = {
                        "id": i + 1,
                        "state": State.STATE_IDLE,
                        "battery": 0.0,
                        "counter": 0,
                        "position": {"x": 0, "y": 0, "z": 0},
                        "phase": 0
                    }
                    self.signals.drone_updated.emit(i, timeout_data)
                    
    def update_drone_display(self, drone_id: int, data: Dict):
        """Update drone display (called from main thread via signal)"""
        if 0 <= drone_id < MAX_COPTERS:
            widget = self.drone_widgets[drone_id]
            
            # Update widget
            widget.set_state(data["state"])
            widget.set_battery(data["battery"])
            widget.set_position(data["position"])
            
            # Update 3D visualization
            self.visualization.update_drone_position(
                drone_id, data["position"], data["state"]
            )
            
            # Record data if experiment is running and recording is active
            if self.experiment_running and self.recorder.is_recording:
                self.recorder.record_drone_data(drone_id, data)
            
    def show_error(self, title: str, message: str):
        """Show error message dialog"""
        QMessageBox.critical(self, title, message)
        
    def toggle_experiment(self):
        """Toggle experiment state"""
        command = {"command": "toggleIsExperimentRunning"}
        try:
            self.command_socket.send_json(command, zmq.NOBLOCK)
            
            # Update experiment state and recording
            self.experiment_running = self.toggle_button.isChecked()
            
            if self.experiment_running:
                # Start recording when experiment starts
                self.recorder.start_recording()
                self.status_label.setText("Recording...")
                self.status_label.setStyleSheet("color: red; font-weight: bold;")
                self.save_button.setEnabled(False)
            else:
                # Stop recording when experiment ends
                if self.recorder.is_recording:
                    self.recorder.stop_recording()
                    self.status_label.setText("Recording Complete - Ready to Save")
                    self.status_label.setStyleSheet("color: green; font-weight: bold;")
                    self.save_button.setEnabled(True)
                    
        except Exception as e:
            print(f"Error sending command: {e}")
    
    def save_experiment(self):
        """Save recorded experiment data to file"""
        if not self.recorder.recorded_data:
            QMessageBox.warning(self, "No Data", "No experiment data to save!")
            return
            
        # Generate default filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"experiment_{timestamp}.json"
        
        # Show save dialog
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Experiment Data",
            default_filename,
            "JSON Files (*.json);;All Files (*)"
        )
        
        if filename:
            if self.recorder.save_to_file(filename):
                QMessageBox.information(
                    self, 
                    "Success", 
                    f"Experiment data saved successfully to:\n{filename}"
                )
                self.status_label.setText("Data Saved")
                self.status_label.setStyleSheet("color: blue; font-weight: bold;")
                self.save_button.setEnabled(False)
            else:
                QMessageBox.critical(
                    self, 
                    "Error", 
                    "Failed to save experiment data!"
                )
    
    def load_experiment(self):
        """Load experiment data from file"""
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Load Experiment Data",
            "",
            "JSON Files (*.json);;All Files (*)"
        )
        
        if filename:
            if self.recorder.load_from_file(filename):
                QMessageBox.information(
                    self, 
                    "Success", 
                    f"Experiment data loaded successfully from:\n{filename}"
                )
                self.status_label.setText("Data Loaded - Ready to Replay")
                self.status_label.setStyleSheet("color: orange; font-weight: bold;")
                self.replay_button.setEnabled(True)
            else:
                QMessageBox.critical(
                    self, 
                    "Error", 
                    "Failed to load experiment data!"
                )
    
    def toggle_replay(self):
        """Start replaying loaded experiment data"""
        if not self.recorder.replay_data:
            QMessageBox.warning(self, "No Data", "No experiment data loaded for replay!")
            return
            
        if self.recorder.start_replay():
            self.status_label.setText("Replaying...")
            self.status_label.setStyleSheet("color: purple; font-weight: bold;")
            self.replay_button.setEnabled(False)
            self.stop_replay_button.setEnabled(True)
            self.replay_timer.start()
        else:
            QMessageBox.warning(self, "Error", "Failed to start replay!")
    
    def stop_replay(self):
        """Stop replaying experiment data"""
        self.recorder.stop_replay()
        self.replay_timer.stop()
        self.status_label.setText("Replay Stopped")
        self.status_label.setStyleSheet("color: blue; font-weight: bold;")
        self.replay_button.setEnabled(True)
        self.stop_replay_button.setEnabled(False)
    
    def _update_replay(self):
        """Update replay visualization (called by timer)"""
        if not self.recorder.is_replaying:
            return
            
        # Get current replay data points
        replay_points = self.recorder.get_replay_data()
        
        # Update visualization with replay data
        for point in replay_points:
            drone_id = point['drone_id']
            if 0 <= drone_id < MAX_COPTERS:
                # Create data dict in expected format
                data = {
                    'state': point['state'],
                    'battery': point['battery'],
                    'position': point['position'],
                    'counter': point['counter'],
                    'phase': point['phase']
                }
                
                # Update display (but don't record - we're in replay mode)
                widget = self.drone_widgets[drone_id]
                widget.set_state(data["state"])
                widget.set_battery(data["battery"])
                widget.set_position(data["position"])
                
                # Update 3D visualization
                self.visualization.update_drone_position(
                    drone_id, data["position"], data["state"]
                )
        
        # Check if replay is finished
        if self.recorder.is_replay_finished():
            self.stop_replay()
            QMessageBox.information(self, "Replay Complete", "Experiment replay has finished!")
    
    def update_decay_time(self, value):
        """Update trail decay time from slider"""
        self.decay_label.setText(f"{value}s")
        if hasattr(self, 'visualization'):
            self.visualization.set_trail_decay_time(float(value))
    
    def update_trail_length(self, value):
        """Update trail length from slider"""
        self.length_label.setText(str(value))
        if hasattr(self, 'visualization'):
            self.visualization.set_max_trail_length(value)
            
    def closeEvent(self, event):
        """Handle application close"""
        # Stop replay timer if running
        if hasattr(self, 'replay_timer'):
            self.replay_timer.stop()
            
        # Stop recording if active
        if hasattr(self, 'recorder') and self.recorder.is_recording:
            self.recorder.stop_recording()
            
        # Stop sniffer thread
        if hasattr(self, 'sniffer_thread'):
            self.sniffer_thread.stop_sniffer()
            self.sniffer_thread.join()
            
        # Close ZMQ sockets
        self.report_socket.close()
        self.command_socket.close()
        self.context.term()
        
        event.accept()


def main():
    """Main application entry point"""
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Create and show main window
    window = SwarmControlTower()
    window.show()
    
    # Run application
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
