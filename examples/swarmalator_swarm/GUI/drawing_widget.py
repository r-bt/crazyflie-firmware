#!/usr/bin/env python3
"""
Drawing Widget for Swarm Path Planning
Allows user to draw paths on x-z plane that the swarm will follow
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QDialog
from PyQt6.QtCore import Qt, QPoint, QPointF, pyqtSignal
from PyQt6.QtGui import QPainter, QPen, QColor, QPainterPath
import numpy as np
from typing import List, Tuple


class DrawingCanvas(QWidget):
    """Canvas widget for drawing paths on x-z plane"""
    
    def __init__(self, width=400, height=400, world_bounds=(-1.3, 1.3, 0.45, 1.2)):
        """
        Args:
            width: Canvas width in pixels
            height: Canvas height in pixels
            world_bounds: (x_min, x_max, z_min, z_max) in meters
        """
        super().__init__()
        self.setFixedSize(width, height)
        self.setStyleSheet("background-color: white; border: 2px solid black;")
        
        self.canvas_width = width
        self.canvas_height = height
        self.world_bounds = world_bounds  # x_min, x_max, z_min, z_max
        
        self.drawing = False
        self.path = QPainterPath()
        self.drawn_points = []  # List of (x, z) in world coordinates
        
    def mousePressEvent(self, event):
        """Start drawing when mouse is pressed"""
        if event.button() == Qt.MouseButton.LeftButton:
            self.drawing = True
            pos = event.position()  # Keep as QPointF
            self.path.moveTo(pos)
            
            # Convert to world coordinates and store
            world_pos = self._pixel_to_world(pos)
            self.drawn_points.append(world_pos)
            self.update()
            
    def mouseMoveEvent(self, event):
        """Continue drawing as mouse moves"""
        if self.drawing:
            pos = event.position()  # Keep as QPointF
            self.path.lineTo(pos)
            
            # Convert to world coordinates and store
            world_pos = self._pixel_to_world(pos)
            self.drawn_points.append(world_pos)
            self.update()
            
    def mouseReleaseEvent(self, event):
        """Stop drawing when mouse is released"""
        if event.button() == Qt.MouseButton.LeftButton:
            self.drawing = False
            
    def paintEvent(self, event):
        """Draw the path on the canvas"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Draw grid
        self._draw_grid(painter)
        
        # Draw the path
        pen = QPen(QColor(0, 0, 255), 3)
        painter.setPen(pen)
        painter.drawPath(self.path)
        
    def _draw_grid(self, painter):
        """Draw a grid with axis labels"""
        pen = QPen(QColor(200, 200, 200), 1)
        painter.setPen(pen)
        
        # Draw vertical lines
        for i in range(5):
            x = i * self.canvas_width // 4
            painter.drawLine(x, 0, x, self.canvas_height)
            
        # Draw horizontal lines
        for i in range(5):
            y = i * self.canvas_height // 4
            painter.drawLine(0, y, self.canvas_width, y)
            
        # Draw axes
        pen = QPen(QColor(0, 0, 0), 2)
        painter.setPen(pen)
        
        # X-axis (horizontal)
        painter.drawLine(0, self.canvas_height // 2, self.canvas_width, self.canvas_height // 2)
        
        # Z-axis (vertical) 
        painter.drawLine(self.canvas_width // 2, 0, self.canvas_width // 2, self.canvas_height)
        
        # Add labels
        painter.drawText(10, self.canvas_height // 2 - 5, "X")
        painter.drawText(self.canvas_width - 20, self.canvas_height // 2 - 5, f"{self.world_bounds[1]}m")
        painter.drawText(self.canvas_width // 2 + 5, 15, "Z")
        painter.drawText(self.canvas_width // 2 + 5, self.canvas_height - 5, f"{self.world_bounds[2]}m")
        
    def _pixel_to_world(self, pixel_pos: QPointF) -> Tuple[float, float]:
        """Convert pixel coordinates to world coordinates (x, z)"""
        x_min, x_max, z_min, z_max = self.world_bounds
        
        # Normalize to [0, 1]
        norm_x = pixel_pos.x() / self.canvas_width
        norm_z = 1.0 - (pixel_pos.y() / self.canvas_height)  # Flip Y axis
        
        # Scale to world coordinates
        world_x = x_min + norm_x * (x_max - x_min)
        world_z = z_min + norm_z * (z_max - z_min)
        
        return (world_x, world_z)
        
    def clear_drawing(self):
        """Clear the current drawing"""
        self.path = QPainterPath()
        self.drawn_points = []
        self.update()
        
    def get_simplified_points(self, num_points=20) -> List[Tuple[float, float, float]]:
        """
        Get simplified path as a list of 3D points (x, y, z) where y=0
        
        Args:
            num_points: Number of points to sample from the path
            
        Returns:
            List of (x, y, z) tuples in world coordinates with y=0
        """
        if not self.drawn_points:
            return []
            
        # Simplify the path by sampling evenly spaced points
        total_points = len(self.drawn_points)
        if total_points <= num_points:
            # If we have fewer points than requested, use all of them
            return [(x, 0.0, z) for x, z in self.drawn_points]
        
        # Sample evenly spaced indices
        indices = np.linspace(0, total_points - 1, num_points, dtype=int)
        sampled_points = [self.drawn_points[i] for i in indices]
        
        # Convert to 3D points with y=0
        return [(x, 0.0, z) for x, z in sampled_points]


class DrawingDialog(QDialog):
    """Dialog window for drawing swarm paths"""
    
    points_confirmed = pyqtSignal(list)  # Emits list of (x, y, z) points
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Draw Swarm Path (X-Z Plane)")
        self.setModal(True)
        
        # Create layout
        layout = QVBoxLayout()
        
        # Instructions
        instructions = QLabel(
            "Draw a path for the swarm to follow:\n"
            "• Draw on the canvas below (X-Z plane, Y=0)\n"
            "• The swarm will follow the path you draw\n"
            "• Click 'Confirm' when done or 'Clear' to start over"
        )
        instructions.setWordWrap(True)
        layout.addWidget(instructions)
        
        # Drawing canvas
        self.canvas = DrawingCanvas()
        layout.addWidget(self.canvas, alignment=Qt.AlignmentFlag.AlignCenter)
        
        # Number of points control
        points_layout = QHBoxLayout()
        points_layout.addWidget(QLabel("Number of waypoints:"))
        
        from PyQt6.QtWidgets import QSpinBox
        self.points_spinbox = QSpinBox()
        self.points_spinbox.setRange(5, 100)
        self.points_spinbox.setValue(20)
        points_layout.addWidget(self.points_spinbox)
        points_layout.addStretch()
        
        layout.addLayout(points_layout)
        
        # Buttons
        button_layout = QHBoxLayout()
        
        clear_button = QPushButton("Clear")
        clear_button.clicked.connect(self.clear_drawing)
        button_layout.addWidget(clear_button)
        
        cancel_button = QPushButton("Cancel")
        cancel_button.clicked.connect(self.reject)
        button_layout.addWidget(cancel_button)
        
        confirm_button = QPushButton("Confirm")
        confirm_button.clicked.connect(self.confirm_drawing)
        confirm_button.setDefault(True)
        button_layout.addWidget(confirm_button)
        
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
        
    def clear_drawing(self):
        """Clear the canvas"""
        self.canvas.clear_drawing()
        
    def confirm_drawing(self):
        """Confirm the drawing and emit the points"""
        num_points = self.points_spinbox.value()
        points = self.canvas.get_simplified_points(num_points)
        
        if not points:
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "No Drawing", "Please draw a path first!")
            return
            
        self.points_confirmed.emit(points)
        self.accept()
