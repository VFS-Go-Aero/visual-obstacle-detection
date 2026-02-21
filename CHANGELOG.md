# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/releases/tag/v0.1.0
