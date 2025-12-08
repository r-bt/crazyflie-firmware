#!/usr/bin/env python3
"""
Drone Widget Module
PyQt widget for displaying individual Crazyflie status and swarmalator parameters
"""

from typing import Dict, Callable, Optional

# PyQt imports
from PyQt6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, 
    QProgressBar, QDoubleSpinBox
)
from PyQt6.QtCore import QTimer, Qt, QThread
from PyQt6.QtGui import QFont

# Local imports
from common import state_dict


class CrazyflieWidget(QGroupBox):
    """PyQt widget for displaying individual Crazyflie status"""
    
    def __init__(self, drone_id: int, parameter_callback: Optional[Callable] = None, parent=None):
        super().__init__(f"Crazyflie #{drone_id}", parent)
        self.drone_id = drone_id
        self.parameter_callback = parameter_callback
        self._setup_ui()
        self.is_active = False
        
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
        
        # Swarmalator parameters section
        params_group = QGroupBox("Swarmalator Parameters")
        params_layout = QGridLayout()
        
        # K parameter (phase synchronization strength)
        params_layout.addWidget(QLabel("K:"), 0, 0)
        self.k_spinbox = QDoubleSpinBox()
        self.k_spinbox.setRange(0.0, 10.0)
        self.k_spinbox.setSingleStep(0.1)
        self.k_spinbox.setValue(0.0)
        self.k_spinbox.setDecimals(2)
        self.k_spinbox.valueChanged.connect(self._on_parameter_changed)
        params_layout.addWidget(self.k_spinbox, 0, 1)
        
        # J parameter (like-attracts-like strength)
        params_layout.addWidget(QLabel("J:"), 1, 0)
        self.j_spinbox = QDoubleSpinBox()
        self.j_spinbox.setRange(0.0, 10.0)
        self.j_spinbox.setSingleStep(0.1)
        self.j_spinbox.setValue(0.0)
        self.j_spinbox.setDecimals(2)
        self.j_spinbox.valueChanged.connect(self._on_parameter_changed)
        params_layout.addWidget(self.j_spinbox, 1, 1)
        
        # Starting phase parameter
        params_layout.addWidget(QLabel("Phase:"), 2, 0)
        self.phase_spinbox = QDoubleSpinBox()
        self.phase_spinbox.setRange(0.0, 6.28)  # 0 to 2π
        self.phase_spinbox.setSingleStep(0.1)
        self.phase_spinbox.setValue(0.0)
        self.phase_spinbox.setDecimals(2)
        self.phase_spinbox.valueChanged.connect(self._on_parameter_changed)
        params_layout.addWidget(self.phase_spinbox, 2, 1)
        
        # Natural frequency parameter
        params_layout.addWidget(QLabel("Nat Freq:"), 3, 0)
        self.nat_freq_spinbox = QDoubleSpinBox()
        self.nat_freq_spinbox.setRange(-5.0, 5.0)
        self.nat_freq_spinbox.setSingleStep(0.1)
        self.nat_freq_spinbox.setValue(0.0)
        self.nat_freq_spinbox.setDecimals(2)
        self.nat_freq_spinbox.valueChanged.connect(self._on_parameter_changed)
        params_layout.addWidget(self.nat_freq_spinbox, 3, 1)
        
        params_group.setLayout(params_layout)
        layout.addWidget(params_group)
        
        self.setLayout(layout)
        
    def set_led(self, color: str):
        """Set LED indicator color"""
        self.led_label.setStyleSheet(f"color: {color}; font-size: 20px;")
        # Store the original color and reset after a delay
        if hasattr(self, '_led_reset_timer'):
            self._led_reset_timer.stop()
        
        # Only create timer if we're in the main thread
        if QThread.currentThread() == self.thread():
            self._led_reset_timer = QTimer()
            self._led_reset_timer.setSingleShot(True)
            self._led_reset_timer.timeout.connect(
                lambda: self.led_label.setStyleSheet("color: grey; font-size: 20px;")
            )
            self._led_reset_timer.start(1000)
        
    def set_state(self, state: int):
        """Update drone state display"""
        if state in state_dict:
            text, color = state_dict[state]
            self.status_label.setText(text)
            self.status_label.setStyleSheet(f"color: {color}; font-weight: bold;")

            if state > 0:
                self.is_active = True
        else:
            self.status_label.setText("ERROR")
            self.status_label.setStyleSheet("color: grey; font-weight: bold;")
            self.is_active = False
            
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
        self.last_position = position  # Store for centroid calculation
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
    
    def _on_parameter_changed(self):
        """Handle parameter changes"""
        if self.parameter_callback:
            parameters = {
                'K': self.k_spinbox.value(),
                'J': self.j_spinbox.value(), 
                'phase': self.phase_spinbox.value(),
                'naturalFrequency': self.nat_freq_spinbox.value()
            }
            self.parameter_callback(self.drone_id, parameters)
    
    def get_parameters(self):
        """Get current parameter values"""
        return {
            'K': self.k_spinbox.value(),
            'J': self.j_spinbox.value(),
            'phase': self.phase_spinbox.value(),
            'naturalFrequency': self.nat_freq_spinbox.value()
        }
    
    def set_parameters(self, params):
        """Set parameter values (without triggering callbacks)"""
        # Temporarily disconnect signals to avoid triggering parameter sends
        self.k_spinbox.valueChanged.disconnect()
        self.j_spinbox.valueChanged.disconnect()
        self.phase_spinbox.valueChanged.disconnect()
        self.nat_freq_spinbox.valueChanged.disconnect()
        
        # Set values
        if 'K' in params:
            self.k_spinbox.setValue(params['K'])
        if 'J' in params:
            self.j_spinbox.setValue(params['J'])
        if 'phase' in params:
            self.phase_spinbox.setValue(params['phase'])
        if 'naturalFrequency' in params:
            self.nat_freq_spinbox.setValue(params['naturalFrequency'])
        
        # Reconnect signals
        self.k_spinbox.valueChanged.connect(self._on_parameter_changed)
        self.j_spinbox.valueChanged.connect(self._on_parameter_changed)
        self.phase_spinbox.valueChanged.connect(self._on_parameter_changed)
        self.nat_freq_spinbox.valueChanged.connect(self._on_parameter_changed)