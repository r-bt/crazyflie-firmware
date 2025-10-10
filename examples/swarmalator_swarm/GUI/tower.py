#!/usr/bin/env python3
"""
PyQt + Vispy GUI for Crazyflie Swarm Control Tower
Combines traditional GUI controls with 3D real-time drone visualization
"""

import sys
import time
import threading
import zmq
import math
from typing import List, Dict

# PyQt imports
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QGridLayout, QLabel, QPushButton, QCheckBox,
    QSplitter, QMessageBox, QGroupBox, QSlider,
    QFileDialog, QScrollArea
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont

# Local imports
from common import MAX_COPTERS, state_dict, State
from sniffer_interface import snifferThread
from experiment_recorder import ExperimentRecorder
from drone_visualization import DroneVisualization
from drone_widget import CrazyflieWidget
from data_signals import DataUpdateSignals

COPTER_ALIVE_TIMEOUT = 2  # sec


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
        self.signals.led_update.connect(self.update_drone_led)
        
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
        # Create scroll area as the main container
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        
        # Create the content widget that will be scrollable
        content_widget = QWidget()
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
        
        # Quick-set parameter controls
        quickset_group = QGroupBox("Quick Parameter Setup")
        quickset_layout = QVBoxLayout()
        
        # Phase setup buttons
        phase_layout = QHBoxLayout()
        phase_layout.addWidget(QLabel("Phase Setup:"))
        
        linear_phase_btn = QPushButton("Linear Spacing")
        linear_phase_btn.clicked.connect(self.set_linear_phases)
        phase_layout.addWidget(linear_phase_btn)
        
        quickset_layout.addLayout(phase_layout)
        
        # Natural frequency setup buttons
        freq_layout = QHBoxLayout()
        freq_layout.addWidget(QLabel("Nat Freq Setup:"))
        
        freq_zero_btn = QPushButton("All 0")
        freq_zero_btn.clicked.connect(self.set_all_freq_zero)
        freq_layout.addWidget(freq_zero_btn)
        
        freq_one_btn = QPushButton("All 1")
        freq_one_btn.clicked.connect(self.set_all_freq_one)
        freq_layout.addWidget(freq_one_btn)
        
        freq_half_btn = QPushButton("Half ±1")
        freq_half_btn.clicked.connect(self.set_half_freq_split)
        freq_layout.addWidget(freq_half_btn)
        
        quickset_layout.addLayout(freq_layout)
        
        quickset_group.setLayout(quickset_layout)
        layout.addWidget(quickset_group)
        
        # Drone status widgets in a grid
        drones_group = QGroupBox("Drone Status")
        drones_layout = QGridLayout()
        
        for i in range(MAX_COPTERS):
            drone_widget = CrazyflieWidget(i + 1, self.send_parameters_to_drone)
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
        content_widget.setLayout(layout)
        
        # Set the content widget in the scroll area
        scroll_area.setWidget(content_widget)
        
        return scroll_area
        
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
                                self.signals.led_update.emit(drone_id, "green")
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
                drone_id, data["position"], data["state"], data["phase"]
            )
            
            # Record data if experiment is running and recording is active
            if self.experiment_running and self.recorder.is_recording:
                self.recorder.record_drone_data(drone_id, data)
            
    def show_error(self, title: str, message: str):
        """Show error message dialog"""
        QMessageBox.critical(self, title, message)
    
    def update_drone_led(self, drone_id: int, color: str):
        """Update drone LED (called from main thread via signal)"""
        if 0 <= drone_id < MAX_COPTERS:
            self.drone_widgets[drone_id].set_led(color)
        
    def toggle_experiment(self, checked: bool):
        command = {"command": "toggleIsExperimentRunning"}
        try:
            self.command_socket.send_json(command, zmq.NOBLOCK)
            
            # Update experiment state and recording
            self.experiment_running = checked
            
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
        """Save experiment data to file"""
        if not self.recorder.recorded_data:
            QMessageBox.warning(self, "No Data", "No experiment data to save!")
            return
            
        filename, _ = QFileDialog.getSaveFileName(
            self, 
            "Save Experiment Data", 
            f"experiment_{int(time.time())}.json",
            "JSON Files (*.json)"
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
            "JSON Files (*.json)"
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
                    drone_id, data["position"], data["state"], data["phase"]
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
    
    def send_parameters_to_drone(self, drone_id: int, parameters: dict):
        """Send parameter updates to a specific drone via ZMQ"""
        try:
            command = {
                "command": "updateSwarmalatorParameters",
                "droneId": drone_id,
                "parameters": parameters
            }
            self.command_socket.send_json(command, zmq.NOBLOCK)
            print(f"Sent parameters to drone {drone_id}: {parameters}")
        except zmq.error.Again:
            print(f"Warning: Could not send parameters to drone {drone_id} - command queue full")
        except Exception as e:
            print(f"Error sending parameters to drone {drone_id}: {e}")
    
    def set_linear_phases(self):
        """Set phases linearly spaced across all active drones"""
        active_drones = []
        
        # Find active drones (assuming drones with ID > 0 are active)
        for i, widget in enumerate(self.drone_widgets):
            if widget.is_active:
                active_drones.append((i, widget))
        
        if not active_drones:
            QMessageBox.warning(self, "No Drones", "No active drones found!")
            return
        
        # Calculate linear spacing: 0 to 2π
        num_drones = len(active_drones)
        phase_step = (2.0 * math.pi) / num_drones
        
        for idx, (drone_idx, widget) in enumerate(active_drones):
            phase = idx * phase_step
            params = {'phase': phase}
            widget.set_parameters(params)
            self.send_parameters_to_drone(widget.drone_id, params)
        
        QMessageBox.information(self, "Success", f"Set linear phases for {num_drones} drones")
    
    def set_all_freq_zero(self):
        """Set all drone natural frequencies to 0"""
        self._set_all_frequencies(0.0)
    
    def set_all_freq_one(self):
        """Set all drone natural frequencies to 1"""
        self._set_all_frequencies(1.0)
    
    def set_half_freq_split(self):
        """Set half drones to +1 and half to -1 natural frequency"""
        active_drones = []
        
        # Find active drones
        for i, widget in enumerate(self.drone_widgets):
            if widget.is_active:
                active_drones.append((i, widget))
        
        if not active_drones:
            QMessageBox.warning(self, "No Drones", "No active drones found!")
            return
        
        num_drones = len(active_drones)
        half_point = num_drones // 2
        
        for idx, (drone_idx, widget) in enumerate(active_drones):
            freq = 1.0 if idx < half_point else -1.0
            params = {'naturalFrequency': freq}
            widget.set_parameters(params)
            self.send_parameters_to_drone(widget.drone_id, params)
        
        QMessageBox.information(self, "Success", 
                               f"Set {half_point} drones to +1 and {num_drones - half_point} drones to -1")
    
    def _set_all_frequencies(self, frequency: float):
        """Helper method to set all drone frequencies to a specific value"""
        count = 0
        for widget in self.drone_widgets:
            if widget.is_active:
                params = {'naturalFrequency': frequency}
                widget.set_parameters(params)
                self.send_parameters_to_drone(widget.drone_id, params)
                count += 1
        
        QMessageBox.information(self, "Success", 
                               f"Set natural frequency to {frequency} for {count} drones")
            
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