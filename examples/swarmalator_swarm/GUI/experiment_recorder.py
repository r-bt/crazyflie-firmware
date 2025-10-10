#!/usr/bin/env python3
"""
Experiment Recorder Module
Handles recording and replaying of experiment data for the Crazyflie Swarm Control Tower
"""

import time
import json
from datetime import datetime
from typing import List, Dict


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