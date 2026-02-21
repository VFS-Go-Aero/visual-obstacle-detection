# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Moved `point_cloud.py` from repository root into the `visual_obstacle_detection` package module and registered it as a `point_cloud` console script entry point.
- Updated `package.xml` with correct version (`0.2.0`), description, maintainer, and license fields.
- Updated `setup.cfg` and `setup.py` with version `0.2.0`, correct maintainer information, and `point_cloud` entry point.
- Simplified `publisher.py` `main()` by inlining node creation into `rclpy.spin()`.

## [0.2.0] - 2026-02-21

### Added

- `point_cloud.py` script with `PointCloud` ROS 2 node that subscribes to registered point cloud topics from two ZED X cameras (`/zed1/...`, `/zed2/...`), converts `PointCloud2` messages to NumPy arrays, and maintains a single merged cloud from both cameras.
- Initial `visual_obstacle_detection` ROS 2 Python package with a `MyPublisher` node, `package.xml`, `setup.py`, `setup.cfg`, and test scaffolding.
- `CONTRIBUTING.md` outlining contribution guidelines and code standards.
- `CHANGELOG.md` to document project updates and versioning.

### Changed

- Renamed `point_cloud_store.py` to `point_cloud.py` and `PointCloudStore` class to `PointCloud`.
- Refactored `PointCloud` class from per-camera dictionary storage to explicit `_cloud1`/`_cloud2` arrays with a merged `cloud` property, replacing the loop-based subscription with dedicated `_cb_zed1`/`_cb_zed2` callbacks and a `_parse`/`_merge` pattern.
- Refactored launch files and `point_cloud.py` for improved readability and PEP 8 compliance.
- Replaced all single-quote strings with double-quote strings across the codebase for consistency.
- Updated `LICENSE` to reflect correct copyright holder and contributors.

### Fixed

- Fixed `main()` to spin `PointCloud` instead of the removed `PointCloudStore`.
- Fixed subscription assignments to store references (`self._sub_zed1`, `self._sub_zed2`) preventing garbage collection.
- Fixed `_parse` return shape with `.reshape(-1, 3)` to ensure a correct `(N, 3)` array even when `read_points` returns a flat list.

## [0.1.0] - 2026-02-18

### Added

- Initial project structure with `.gitignore`.
- MIT License.
- Launch file for dual ZED X camera setup with static transform between camera frames (`multi_zed_tf.launch.py`).
- Launch file for dual ZED X camera setup without transform (`multi_zed.launch.py`).

### Changed

- Updated static transform frame arguments from `left_camera_frame` to `camera_link`.
- Updated ZED2 static transform offset to `(0.0, -0.31, 0.0)` with zero rotation.

[Unreleased]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/VFS-Go-Aero/visual-obstacle-detection/releases/tag/v0.1.0
