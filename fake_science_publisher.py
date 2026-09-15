#!/usr/bin/env python3
"""
fake_science_publisher.py

Simulates the rover side of the URC Science Mission:
- Publishes live instrument readings on /science_data (temp, humidity, pH,
  a life-detection signal, GNSS coordinates, and current site ID)
- Listens for operator commands on /sample_command
- Runs a simple state machine (IDLE -> DRILLING -> ANALYZING -> COMPLETE)
  and publishes state changes on /sample_status, so the GUI has something
  real to react to instead of a command just disappearing into the void
"""

import json
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# How long each stage of the fake sample-collection process takes
DRILLING_DURATION_SEC = 4.0
ANALYZING_DURATION_SEC = 4.0
COMPLETE_HOLD_SEC = 6.0  # how long to show COMPLETE before resetting to IDLE


class FakeSciencePublisher(Node):
    def __init__(self):
        super().__init__('fake_science_publisher')

        self.data_publisher = self.create_publisher(String, '/science_data', 10)
        self.status_publisher = self.create_publisher(String, '/sample_status', 10)
        self.command_subscription = self.create_subscription(
            String, '/sample_command', self.command_callback, 10
        )

        # Instrument state
        self.temperature = 18.0
        self.humidity = 32.0
        self.ph = 7.1
        self.life_signal = 0.15  # 0.0-1.0, a fake biosignature reading
        self.sites = ['Site A', 'Site B', 'Site C']
        self.site_index = 0
        self.base_lat = 38.4064
        self.base_lon = -110.7922  # roughly MDRS, Utah

        # Sample state machine
        self.sample_state = 'IDLE'
        self.state_entered_at = time.time()
        self.sample_mass_g = None

        self.data_timer = self.create_timer(1.0, self.publish_science_data)
        self.state_timer = self.create_timer(0.5, self.advance_state_machine)

        self.get_logger().info('Fake science node started (/science_data, /sample_status)')

    # --- instrument data -------------------------------------------------

    def publish_science_data(self):
        self.temperature += random.uniform(-0.3, 0.3)
        self.humidity += random.uniform(-0.5, 0.5)
        self.ph += random.uniform(-0.05, 0.05)
        self.life_signal = max(0.0, min(1.0, self.life_signal + random.uniform(-0.03, 0.05)))

        self.humidity = max(0.0, min(100.0, self.humidity))
        self.ph = max(0.0, min(14.0, self.ph))

        # Occasionally "move" to a new site to simulate the rover roving
        if random.random() < 0.05:
            self.site_index = (self.site_index + 1) % len(self.sites)

        payload = {
            'site_id': self.sites[self.site_index],
            'gnss_lat': round(self.base_lat + random.uniform(-0.0006, 0.0006), 6),
            'gnss_lon': round(self.base_lon + random.uniform(-0.0006, 0.0006), 6),
            'temperature_c': round(self.temperature, 2),
            'humidity_pct': round(self.humidity, 2),
            'ph': round(self.ph, 2),
            'life_detection_signal': round(self.life_signal, 3),
            'timestamp': time.time(),
        }

        msg = String()
        msg.data = json.dumps(payload)
        self.data_publisher.publish(msg)

    # --- command handling / state machine --------------------------------

    def command_callback(self, msg):
        try:
            cmd = json.loads(msg.data).get('command')
        except json.JSONDecodeError:
            self.get_logger().warn('Received malformed command message')
            return

        self.get_logger().info(f'Received command: {cmd}')

        if cmd == 'BEGIN_SAMPLE_COLLECTION' and self.sample_state == 'IDLE':
            self._set_state('DRILLING')
        elif cmd == 'ABORT':
            self._set_state('IDLE')
            self.sample_mass_g = None

    def _set_state(self, new_state):
        self.sample_state = new_state
        self.state_entered_at = time.time()
        self.get_logger().info(f'Sample state -> {new_state}')
        self.publish_status()

    def advance_state_machine(self):
        elapsed = time.time() - self.state_entered_at

        if self.sample_state == 'DRILLING' and elapsed >= DRILLING_DURATION_SEC:
            self._set_state('ANALYZING')
        elif self.sample_state == 'ANALYZING' and elapsed >= ANALYZING_DURATION_SEC:
            self.sample_mass_g = round(random.uniform(5.0, 12.0), 1)
            self._set_state('COMPLETE')
        elif self.sample_state == 'COMPLETE' and elapsed >= COMPLETE_HOLD_SEC:
            self.sample_mass_g = None
            self._set_state('IDLE')

    def publish_status(self):
        payload = {
            'state': self.sample_state,
            'sample_mass_g': self.sample_mass_g,
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeSciencePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
