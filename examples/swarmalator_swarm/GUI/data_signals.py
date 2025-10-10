#!/usr/bin/env python3
"""
Data Signals Module
Qt signals for thread-safe GUI updates in the Crazyflie Swarm Control Tower
"""

from PyQt6.QtCore import QObject, pyqtSignal


class DataUpdateSignals(QObject):
    """Qt signals for thread-safe GUI updates"""
    drone_updated = pyqtSignal(int, dict)  # drone_id, data
    error_occurred = pyqtSignal(str, str)  # title, message
    led_update = pyqtSignal(int, str)  # drone_id, color