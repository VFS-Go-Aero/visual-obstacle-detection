"""FLIGHT TEST: full stack, no RViz, no sphere. STRICT: if cameras, /merged_cloud, or the
flight controller link don't come up, prints NOT SAFE TO FLY and shuts everything down.

    ros2 launch startup_launcher flight.launch.py
    ros2 launch startup_launcher flight.launch.py fcu_url:=/dev/ttyACM0:57600

Ctrl+C stops everything. Leftovers: ros2 run startup_launcher stop
"""
from startup_launcher.stack import generate


def generate_launch_description():
    return generate('flight')
