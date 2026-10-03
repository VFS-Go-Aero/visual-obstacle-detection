"""GROUND TEST: full stack + orange camera sphere + RViz. Failed checks WARN and keep going.

    ros2 launch startup_launcher ground.launch.py
    ros2 launch startup_launcher ground.launch.py fcu_url:=/dev/ttyACM0:57600

Ctrl+C stops everything. Leftovers: ros2 run startup_launcher stop
"""
from startup_launcher.stack import generate


def generate_launch_description():
    return generate('ground')
