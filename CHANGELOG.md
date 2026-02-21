# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2026-02-21

### Added

- `PointCloud` ROS 2 node that subscribes to registered point cloud topics from two ZED X cameras, converts `PointCloud2` messages to NumPy arrays, and maintains a merged cloud from both cameras.
- Point cloud merging from dual ZED X camera subscriptions (`zed1`, `zed2`).
- Initial `visual_obstacle_detection` ROS 2 package with `my_first_pkg` publisher node, `setup.py`, `package.xml`, and test scaffolding.
- `CONTRIBUTING.md` outlining contribution guidelines and code standards.

### Changed

- Renamed `PointCloudStore` class to `PointCloud` and updated initialization parameters.
- Refactored `PointCloud` class to maintain a single merged cloud from two ZED X cameras.
- Refactored launch files and point cloud script for improved readability and PEP 8 compliance.
- Replaced all single-quote strings with double-quote strings for consistency.
- Updated `LICENSE` to reflect correct copyright holder and contributors.

### Fixed

- Fixed `main` function to spin the `PointCloud` class instead of the old `PointCloudStore`.
- Fixed subscription assignments to store references (`_sub_zed1`, `_sub_zed2`) preventing garbage collection.
- Fixed point cloud data reshape to ensure correct `(N, 3)` array shape from `_parse`.

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
