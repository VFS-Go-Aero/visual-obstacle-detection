# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Added `launch_files/launch/launch_all.launch.py` to launch MAVROS, the multi-ZED setup, and the visual obstacle detection pipeline together.
- Added `scripts/launch.sh` shell helper to start the full system with optional ROS logging enabled.

### Changed
- Updated `launch_files/launch/multi_zed.launch.py` and `launch_files/launch/single_zed.launch.py` to use `info` logging level for ZED driver launches and static transform publishers.

## [0.10.3] - 2026-04-18

### Added
- Added `v` launch argument to `visual_obstacle_detection/launch/visual_obstacle_detection.launch.py` to enable verbose obstacle detection logging via `v:=true`.

## [0.10.2] - 2026-04-18

### Changed
- Updated `launch_files/launch/multi_zed.launch.py` to correct static extrinsic rotations for `zed1_camera_link` and `zed2_camera_link` relative to `base_link` to no rotation.

## [0.10.1] - 2026-04-15

### Added
- Added `visual_obstacle_detection/launch/visual_obstacle_detection.launch.py` to launch the full visual obstacle detection pipeline without needing individual launches for each package.

### Changed
- Updated package dependency metadata in `visual_obstacle_detection/package.xml`.

## [0.10.0] - 2026-04-10

### Added

- Added `launch_files/launch/single_zed.launch.py` for single-camera testing (zed1 only), enabling graceful operation when only one ZED camera is available.
- Added XML-configurable exclusion boxes for merged cloud filtering in `visual_obstacle_detection/visual_obstacle_detection/point_cloud.py`, with a default `excluded_boxes.xml` in the same module directory; points inside configured axis-aligned boxes in `base_link` (or configured target frame) are excluded from `/merged_cloud` publication.

### Changed

- Updated `visual_obstacle_detection/visual_obstacle_detection/obstacle_to_mavlink.py` to convert obstacle point coordinates from the node's incoming body-centered RFU convention (`x=right, y=forward, z=up`) into MAVLink body FRD (`x=forward, y=right, z=down`) before publishing `ObstacleDistance3D.position`.
- Updated `visual_obstacle_detection/visual_obstacle_detection/obstacle_to_mavlink.py` to publish `ObstacleDistance3D.frame` as explicit `MAV_FRAME_BODY_FRD` (`12`) instead of `0` (MAVROS default frame), ensuring body-relative obstacle data for ArduPilot.
- Added `launch_files/launch/single_zed.launch.py` single ZED launch file with static transform support from `base_link` to set camera link, intended for use in single-camera drone.

## [0.9.0] - 2026-03-31

### Added

- Added `launch_files/launch_files/drone_pose.py` node publishing `/drone_pose` from `/mavros/local_position/odom`.
- Added additional unit tests and linting checks in `launch_files` package: `test_flake8`, `test_pep257`, and `test_copyright` coverage.

### Changed

- Refactored `launch_files` package layout for Python/ROS 2 ament packaging, relocating launch scripts under `launch_files/launch` and updating package metadata (`package.xml`, `setup.py`, `__init__.py`).
- Reworked `launch_files/launch/multi_zed.launch.py` to launch two explicit `zed_wrapper` camera instances (`zed1`, `zed2`) with TF publishing disabled, and to define static extrinsics from `base_link` to each camera link in the launch file.
- Updated `visual_obstacle_detection/visual_obstacle_detection/point_cloud.py` to default to direct `zed_wrapper` point cloud topics (`/zed1/zed_node/point_cloud/cloud_registered`, `/zed2/zed_node/point_cloud/cloud_registered`), support target-frame TF lookup, and maintain merged-cloud publication in the configured target frame.
- Updated `visual_obstacle_detection/visual_obstacle_detection/obstacle_detection.py` to consume `/merged_cloud` directly, preserve incoming frame IDs, and publish obstacle cloud in the current frame.
- Updated documentation guidance in `CONTRIBUTING.md` and `bash-commands/README.md` for the new `launch_files` package and the single `multi_zed.launch.py` entry point.

### Fixed

- Fixed flake8 style issues in `drone_pose.py` (blank lines, line length, trailing whitespace, final newline).
- Fixed `launch_files` packaging to install only launch Python files (`launch/*.py`), avoiding build failures from `launch/__pycache__` paths.
- Improved merge-node TF robustness by using latest available transform lookup for point cloud conversion, reducing timestamp extrapolation failures during live operation.

### Removed

- Removed legacy launch files `launch_files/launch/multi_zed_odom.launch.py` and `launch_files/launch/multi_zed_tf.launch.py` to keep `launch_files/launch/multi_zed.launch.py` as the single supported dual-camera launch entry point.

## [0.8.1] - 2026-03-11

### Removed

- Removed deprecated `scan_to_mavlink` package and all associated files (`scan_to_mavlink_node.py`, `setup.py`, `setup.cfg`, `package.xml`, tests, and resource).
- Removed deprecated test scripts `scripts/fakedata.py` (fake obstacle publisher) and `scripts/testmav.py` (MAVLink test).
- Removed `scan_to_mavlink` test step from the CI workflow (`.github/workflows/ros2_ci.yml`).

## [0.8.0] - 2026-03-11

### Added

- New `obstacle_to_mavlink` module (`obstacle_to_mavlink.py`) with an `ObstacleToMavlink` node that subscribes to the obstacle point cloud on `/merged_cloud/obstacles`, reads obstacle points with `obstacle_id` fields, and publishes each obstacle as an `ObstacleDistance3D` message to `/mavros/obstacle_distance_3d/send` at a configurable frequency with configurable distance parameters (`MAX_DISTANCE`, `MIN_DISTANCE`, `FREQUENCY`).
- `obstacle_to_mavlink` console script entry point in `setup.py`.
- `frame_id` (`zed1_left_camera_frame`) set on the obstacle cloud header in `obstacle_detection.py`, required for visualization in RViz.

### Changed

- `build_sector_map()` now returns a `(winner_mask, winner_sector)` tuple, providing the sector ID for each obstacle representative point in `obstacle_detection.py`.
- Reduced sector-map resolution from 32×16 to 8×8 azimuth/elevation bins in `obstacle_detection.py`.
- Obstacle cloud `PointField` renamed from `rgb` to `obstacle_id`, now carrying an integer sector ID instead of a packed red RGB value in `obstacle_detection.py`.
- Sector-map obstacle count log level changed from `debug` to `info` in `obstacle_detection.py`.

### Removed

- Removed `obstacle_detection_pc` module (`obstacle_detection_pc.py`) and its `obstacle_detection_pc` console script entry point from `setup.py`.

### Fixed

- Removed extra blank line in `scan_to_mavlink_node.py`.

## [0.7.0] - 2026-03-04

### Added

- New `obstacle_detection_pc` module (`obstacle_detection_pc.py`) with a `PointCloud` node providing distance/bounding-box-based obstacle filtering, RGB coloring of obstacle vs. non-obstacle points, and 3×3 region visualization using `MarkerArray`.
- `obstacle_detection_pc` console script entry point in `setup.py`.
- `build_sector_map()` function in `obstacle_detection.py` — sector-based obstacle detection that divides the point cloud into azimuth/elevation bins and finds the nearest dense distance-shell in each sector.
- Configurable sector-map parameters (`N_AZ`, `N_EL`, `DIST_BIN_W`, `MIN_POINTS`) in `obstacle_detection.py`.
- NaN and zero-distance point filtering in sector-map construction.
- Source command section in `bash-commands/README.md`.

### Changed

- Renamed `PointCloud` class to `ObstacleDetection` and node name to `obstacle_detection_segment` in `obstacle_detection.py`.
- Replaced simple distance/height threshold obstacle filtering with sector-map-based detection in `obstacle_detection.py`.
- Obstacle cloud on `/merged_cloud/obstacles` now publishes only obstacle-representative points (colored red) instead of the full cloud with per-point coloring.
- Updated `main()` in `obstacle_detection.py` to instantiate `ObstacleDetection` and properly destroy the node before shutdown.
- Switched `point_cloud2` import from `from sensor_msgs_py import point_cloud2` to `import sensor_msgs_py.point_cloud2 as pc2` in `obstacle_detection.py`.
- Reformatted code in `obstacle_detection.py` for consistency and readability.

### Removed

- Removed `publisher.py` (`MyPublisher` node) and its `publisher` console script entry point from `setup.py`.

## [0.6.0] - 2026-02-28

### Added

- Added `obstacle_detection` console script entry point for the new obstacle detection node.
- Merged point cloud publisher on `/merged_cloud` topic in the `PointCloud` node using `create_cloud_xyz32`.

### Changed

- Updated `CONTRIBUTING.md` to document the new package structure and add release/versioning instructions.

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

[Unreleased]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.10.3...HEAD
[0.10.3]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.10.2...v0.10.3
[0.10.2]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.10.1...v0.10.2
[0.10.1]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.10.0...v0.10.1
[0.10.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.9.0...v0.10.0
[0.9.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.8.1...v0.9.0
[0.8.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.4.4...v0.5.0
[0.4.3]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.4.2...v0.4.3
[0.4.2]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.4.1...v0.4.2
[0.4.1]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.4.0...v0.4.1
[0.4.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.3.1...v0.4.0
[0.3.1]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/releases/tag/v0.1.0
