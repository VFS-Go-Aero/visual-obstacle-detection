# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Added `obstacle_detection` console script entry point for the new obstacle detection node.
- Merged point cloud publisher on `/merged_cloud` topic in the `PointCloud` node using `create_cloud_xyz32`.

### Fixed

- Corrected indentation for publisher initialization in `point_cloud.py`.
- Corrected indentation and minor formatting issues in `point_cloud.py`.
- Renamed console script from `obstacle_detector` to `obstacle_detection` and updated node name accordingly.

## [0.5.0] - 2026-02-28

### Added

- Created new `launch_files` ROS 2 package with `package.xml` and moved launch scripts into `launch_files/launch/`.
  - Included dual-camera ZED launch files: `multi_zed.launch.py` and `multi_zed_tf.launch.py`.

### Changed

- Relocated existing launch files from the repository root into the new `launch_files/launch` directory for proper packaging.
- Updated ROS2 CI workflow to run tests for the new `launch_files` package alongside existing packages.

## [0.4.4] - 2026-02-27

### Changed

- Enhanced `CONTRIBUTING.md` with additional PR standards, development workflow, and prohibited actions to improve collaboration and maintain code quality.
- Reorganized `package.xml` by separating repository and website URLs from maintainer information.
- Removed `COLCON_IGNORE` step for `scan_to_mavlink` in CI workflow.
- Added README and updated package metadata with maintainer URLs and authors.
- Updated `README.md` with detailed project overview, organization information, contact details, and social media links.

## [0.4.3] - 2026-02-27

### Changed

- Moved `fakedata.py` and `testmav.py` to a new `scripts/` folder for better organization.
- Flattened the `scan_to_mavlink` package structure by removing the redundant nested folder.
- Updated ROS2 CI workflow to test both `visual_obstacle_detection` and `scan_to_mavlink` packages.
- Refactored and reformatted `scan_to_mavlink` to adhere to the repository's contributing guidelines and pass all test standards.

## [0.4.2] - 2026-02-26

### Added

- ROS 2 CI workflow (`.github/workflows/ros2_ci.yml`) using `ros-industrial/industrial_ci` to build and test the package on every push/PR to `main` and `dev`.
- CI step to place a `COLCON_IGNORE` marker in `scan_to_mavlink/` so colcon skips that package during CI builds.

### Fixed

- Removed trailing blank line in `setup.py` (flake8 W391).
- Reformatted multi-line docstrings in `point_cloud.py` and `publisher.py` to comply with `ament_pep257` conventions (D205, D213, D400, D413).

### Changed

- Updated `CONTRIBUTING.md` with full repository layout, ROS 2 package guidelines, CI documentation, and corrected docstring formatting rules.
- Added Docker layer caching to the ROS 2 CI workflow to speed up builds by caching the base image (`ros:humble-ros-base-jammy`).
- Updated `.gitignore` to exclude `/tmp/` to prevent Docker cache files from being tracked.
- Updated `.github/workflows/ros2_ci.yml` to prevent redundant CI runs after pull request merges. The workflow now runs for direct pushes and non-pull-request merges, but skips redundant runs if already executed during a pull request.

## [0.4.1] - 2026-02-26

### Added

- Absorbed `bash-commands` repository, adding a reference README with common ROS 2 CLI commands (rangefinder, LaserScan, etc.).

## [0.4.0] - 2026-02-25

### Added

- Absorbed `scan_to_mavlink` ROS 2 package from the external `ros2-pkgs` repository, adding MAVLink obstacle-distance message support via `scan_to_mavlink_node`.
- Launch files, test scaffolding, and helper scripts (`fakedata.py`, `testmav.py`) for the `scan_to_mavlink` package.

## [0.3.1] - 2026-02-25

### Fixed

- Fixed `_parse` dtype cast error by extracting structured array fields individually instead of casting directly to `float32`.

## [0.3.0] - 2026-02-23

### Added

- Initial `visual_obstacle_detection` ROS 2 Python package with a `MyPublisher` node, `package.xml`, `setup.py`, `setup.cfg`, and test scaffolding.
- `CHANGELOG.md` to document project updates and versioning.
- `CONTRIBUTING.md` outlining contribution guidelines and code standards.

### Changed

- Replaced all single-quote strings with double-quote strings across the codebase for consistency.
- Updated `LICENSE` to reflect correct copyright holder (Vertical Flight Systems Purdue) and contributors.
- Refactored launch files and `point_cloud.py` for improved readability and PEP 8 compliance.
- Moved `point_cloud.py` from repository root into the `visual_obstacle_detection` package module and registered it as a `point_cloud` console script entry point.
- Updated `package.xml` with correct version (`0.2.0`), description, maintainer, and license fields.
- Updated `setup.cfg` and `setup.py` with version `0.2.0`, correct maintainer information, and `point_cloud` entry point.
- Simplified `publisher.py` `main()` by inlining node creation into `rclpy.spin()`.

### Fixed

- Fixed subscription assignments to store references (`self._sub_zed1`, `self._sub_zed2`) preventing garbage collection.
- Fixed `_parse` return shape with `.reshape(-1, 3)` to ensure a correct `(N, 3)` array even when `read_points` returns a flat list.

## [0.2.0] - 2026-02-21

### Added

- `PointCloud` ROS 2 node that subscribes to registered point cloud topics from two ZED X cameras, converts `PointCloud2` messages to NumPy arrays, and maintains a merged cloud from both cameras.
- Point cloud merging from dual ZED X camera subscriptions (`zed1`, `zed2`).

### Changed

- Renamed `PointCloudStore` class to `PointCloud` and updated initialization parameters.
- Refactored `PointCloud` class to maintain a single merged cloud from two ZED X cameras.

### Fixed

- Fixed `main` function to spin the `PointCloud` class instead of the old `PointCloudStore`.

## [0.1.0] - 2026-02-18

### Added

- Initial project commit.
- MIT License.
- Launch file for multi-camera ZED setup with static transform (`multi_zed_tf.launch.py`).
- Launch file for multi-camera ZED setup without transform (`multi_zed.launch.py`).
- Static transform publisher for ZED camera coordinate frames.

### Changed

- Updated ZED2 transform parameters to zero.
- Updated static transform publisher arguments for clarity.

[Unreleased]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.5.0...HEAD
[0.5.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.4.4...v0.5.0
[0.4.3]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.4.2...v0.4.3
[0.4.2]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.4.1...v0.4.2
[0.4.1]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.4.0...v0.4.1
[0.4.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.3.1...v0.4.0
[0.3.1]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/releases/tag/v0.1.0
