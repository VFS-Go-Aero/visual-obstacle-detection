# Visual Obstacle Detection

[![License: MIT](https://img.shields.io/badge/License-MIT-red.svg)](LICENSE) [![Changelog](https://img.shields.io/badge/changelog-latest-green.svg)](CHANGELOG.md) [![Contributing](https://img.shields.io/badge/contributing-guidelines-blue.svg)](CONTRIBUTING.md)

## Overview
Visual Obstacle Detection is a project designed to detect obstacles using visual data. It leverages advanced algorithms to process point clouds and publish relevant information for use within the **Human Emergency Aerial Rescue and Transport (HEART)** Aircraft. This project is a critical component of the perception stack for the HEART Drone, enabling it to navigate complex environments safely and efficiently.

## Features
- Point cloud processing and merging from dual ZED X cameras
- Sector-map-based obstacle detection dividing the point cloud into azimuth/elevation bins
- Distance/bounding-box obstacle filtering with RGB coloring and 3×3 region `MarkerArray` visualization
- MAVLink obstacle-distance message conversion via `scan_to_mavlink`
- ROS 2 (Humble) integration with multiple `ament_python` packages

## Repository Structure
- `visual_obstacle_detection/`: Core ROS 2 package for obstacle detection
  - `point_cloud.py` — Point cloud subscriber and merger
  - `obstacle_detection.py` — Sector-map-based obstacle detection node
  - `obstacle_detection_pc.py` — Distance/bounding-box obstacle filtering with `MarkerArray` visualization
- `scan_to_mavlink/`: Converts LaserScan data to MAVLink obstacle-distance messages
- `launch_files/`: ROS 2 launch files for multi-camera ZED setup
- `scripts/`: Utility scripts for testing and data generation
- `bash-commands/`: Common ROS 2 CLI commands reference

## Organization
This project is maintained by **Vertical Flight Systems Purdue**, an organization dedicated to advancing aerospace technologies through innovative software solutions. Our mission is to create reliable and efficient tools for robotics and aerospace applications. Learn more about us [here](https://vfspurdue.com/about-us/).

### Contact
For inquiries, please reach out to us at:
- **Email**: contact@vfspurdue.com
- **Website**: [vfspurdue.com](https://vfspurdue.com)
- **Location**: Purdue University, West Lafayette, IN, USA

### Social Media
Stay connected with us on social media:
- [LinkedIn](https://www.linkedin.com/company/vfspurdue)
- [Instagram](https://www.instagram.com/vfspurdue/)

## Getting Started
To get started, clone the repository and follow the setup instructions in the respective `README.md` files within each module.