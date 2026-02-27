# Contributing

Thank you for contributing to **visual-obstacle-detection**. Follow the guidelines below to keep the codebase consistent and review-friendly.

## Getting Started

1. Fork or clone the repository.
2. Create a feature branch from `main`:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. Make your changes, commit, and open a pull request.

## Repository Layout

```
visual-obstacle-detection/
├── .github/
│   └── workflows/
│       └── ros2_ci.yml          # ROS 2 CI workflow (industrial_ci)
├── CHANGELOG.md
├── CONTRIBUTING.md
├── LICENSE
├── launch_files/                # ROS 2 launch files
│   ├── multi_zed.launch.py
│   └── multi_zed_tf.launch.py
├── scripts/                     # Utility scripts
│   ├── fakedata.py              # ROS 2 fake obstacle publisher
│   └── testmav.py               # MAVLink fake obstacle sender
├── scan_to_mavlink/             # ROS 2 ament_python package
│   ├── package.xml              # Package manifest (dependencies, metadata)
│   ├── setup.py                 # Python package setup
│   ├── setup.cfg                # Entry-point script directories
│   ├── resource/                # ament index marker
│   │   └── scan_to_mavlink
│   ├── scan_to_mavlink/         # Package source code
│   │   ├── __init__.py
│   │   └── scan_to_mavlink_node.py
│   └── test/                    # ament lint + pytest tests
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
├── visual_obstacle_detection/   # ROS 2 ament_python package
│   ├── package.xml              # Package manifest (dependencies, metadata)
│   ├── setup.py                 # Python package setup
│   ├── setup.cfg                # Entry-point script directories
│   ├── resource/                # ament index marker
│   │   └── visual_obstacle_detection
│   ├── visual_obstacle_detection/
│   │   ├── __init__.py
│   │   ├── point_cloud.py       # Point cloud subscriber node
│   │   └── publisher.py         # Example publisher node
│   └── test/                    # ament lint + pytest tests
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
```

## ROS 2 Package Guidelines

This project is built as a **ROS 2 Humble** `ament_python` package.

### Building

```bash
# From your colcon workspace src/ directory:
cd ~/colcon_ws/src
ln -s /path/to/visual-obstacle-detection/visual_obstacle_detection .

# Build
cd ~/colcon_ws
colcon build --packages-select visual_obstacle_detection
source install/setup.bash
```

### Adding Dependencies

- **Python runtime dependencies** — Add `<depend>` or `<exec_depend>` entries in `package.xml`.
- **Test dependencies** — Add `<test_depend>` entries in `package.xml`.
- **pip-only packages** — Add them to `install_requires` in `setup.py` as well.

### Adding Nodes

1. Create a new module under `visual_obstacle_detection/visual_obstacle_detection/`.
2. Implement a `main()` entry point.
3. Register the entry point in `setup.py` under `console_scripts`:
   ```python
   "my_node = visual_obstacle_detection.my_module:main",
   ```

### Launch Files

Launch files live in the top-level `launch_files/` directory and use the ROS 2 Python launch format (`.launch.py`).

## Code Style

This project follows [PEP 8](https://peps.python.org/pep-0008/). All Python code must conform to the standards below.

### Formatting

- **Indentation** — Use 4 spaces per level. Never use tabs.
- **Maximum line length** — 79 characters for code, 72 for docstrings and comments.
- **Blank lines** — Two blank lines before and after top-level definitions (classes, functions). One blank line between methods inside a class.
- **Encoding** — Use UTF-8. Omit the encoding declaration (Python 3 default).
- **No trailing blank lines** — Files must not end with extra blank lines (flake8 W391).

### Imports

- Place all imports at the top of the file, after any module docstring.
- Group imports in this order, separated by a blank line:
  1. Standard library (`os`, `sys`, `typing`, …)
  2. Third-party packages (`numpy`, `rclpy`, …)
  3. Local / project-specific modules
- Use absolute imports. Avoid wildcard imports (`from module import *`).

### Naming Conventions

| Element             | Convention          | Example                |
|---------------------|---------------------|------------------------|
| Modules / packages  | `snake_case`        | `point_cloud.py`       |
| Classes             | `PascalCase`        | `PointCloud`           |
| Functions / methods | `snake_case`        | `merge_clouds()`       |
| Constants           | `UPPER_SNAKE_CASE`  | `MAX_RANGE`            |
| Private members     | `_leading_underscore` | `_cloud1`            |

### Whitespace

- No trailing whitespace on any line.
- Use a single space around binary operators (`=`, `+=`, `==`, `and`, etc.).
- No space immediately inside parentheses, brackets, or braces.
- No space before a colon in slices: `cloud[1:3]`, not `cloud[1 : 3]`.

### Strings

- Use double quotes (`"`) for strings by default to stay consistent with the existing codebase.
- Use triple double-quoted strings (`"""`) for docstrings.

### Docstrings

- Every public module, class, and function must have a docstring ([PEP 257](https://peps.python.org/pep-0257/)).
- The summary line must end with a period.
- For multi-line docstrings, the summary must start on the **second line** (the line after `"""`), not on the same line as the opening quotes (D213).
- Leave a **blank line** between the summary and the description (D205).
- Leave a **blank line after the last section** before the closing `"""` (D413).
- Use the following format:
  ```python
  """
  Merge two point clouds into a single array.

  Concatenates cloud_a and cloud_b along axis 0 and returns
  the result as a contiguous float32 array.

  Parameters
  ----------
  cloud_a : np.ndarray
      First cloud, shape (M, 3).
  cloud_b : np.ndarray
      Second cloud, shape (K, 3).

  Returns
  -------
  np.ndarray
      Merged cloud, shape (M + K, 3).

  """
  ```
- Single-line docstrings keep everything on one line:
  ```python
  """Spin the PointCloud node until shutdown."""
  ```

### Type Hints

- Add type hints to function signatures where practical.
  ```python
  def get_points(msg: PointCloud2) -> np.ndarray:
      ...
  ```

### Comments

- Use comments sparingly — prefer clear code and docstrings.
- Write comments as complete sentences, starting with a capital letter.
- Use inline comments only when the intent is not obvious from the code itself.

## Linting & CI

### Automated CI

Every push and pull request to `main` or `dev` triggers the **ROS2 CI** workflow (`.github/workflows/ros2_ci.yml`). It uses [`ros-industrial/industrial_ci`](https://github.com/ros-industrial/industrial_ci) to build and test the package inside a ROS 2 Humble Docker container.

The CI runs three ament lint tests automatically:

| Test | What it checks |
|------|----------------|
| `ament_copyright` | Copyright header present in source files |
| `ament_flake8` | PEP 8 compliance (via flake8) |
| `ament_pep257` | Docstring conventions (via pydocstyle) |

**Your PR will not pass CI unless all three linters report zero errors.**

### Running Linters Locally

```bash
cd visual_obstacle_detection

# flake8 (PEP 8)
flake8 --max-line-length 79 .

# pydocstyle (PEP 257)
pydocstyle .

# Or run the full colcon test suite (requires a colcon workspace):
cd ~/colcon_ws
colcon build --packages-select visual_obstacle_detection
colcon test --packages-select visual_obstacle_detection
colcon test-result --verbose
```

Fix all warnings and errors before submitting.

## Commits

- Write clear, imperative-mood commit messages (e.g., *Add point cloud merging*, not *Added point cloud merging*).
- Keep commits focused — one logical change per commit.

## Pull Requests

- Reference any related issues in the PR description.
- Ensure the ROS2 CI workflow passes (all ament lint tests green).
- Request a review from at least one maintainer.
- Update [CHANGELOG.md](CHANGELOG.md) under the `[Unreleased]` section with a summary of your changes.

## License

By contributing, you agree that your contributions will be licensed under the project's [MIT License](LICENSE).
