# Contributing

Thank you for contributing to **visual-obstacle-detection**. Follow the guidelines below to keep the codebase consistent and review-friendly.

## Getting Started

1. Fork or clone the repository.
2. Create a feature branch from `main`:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. Make your changes, commit, and open a pull request.

## Code Style

This project follows [PEP 8](https://peps.python.org/pep-0008/). All Python code must conform to the standards below.

### Formatting

- **Indentation** — Use 4 spaces per level. Never use tabs.
- **Maximum line length** — 79 characters for code, 72 for docstrings and comments.
- **Blank lines** — Two blank lines before and after top-level definitions (classes, functions). One blank line between methods inside a class.
- **Encoding** — Use UTF-8. Omit the encoding declaration (Python 3 default).

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
- Use the following format:
  ```python
  def merge_clouds(cloud_a, cloud_b):
      """Merge two point clouds into a single (N, 3) array.

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

## Linting

Run the following before opening a pull request:

```bash
# Check style compliance
flake8 --max-line-length 79 .

# Auto-format (optional but recommended)
black --line-length 79 .
```

Fix all warnings and errors before submitting.

## Commits

- Write clear, imperative-mood commit messages (e.g., *Add point cloud merging*, not *Added point cloud merging*).
- Keep commits focused — one logical change per commit.

## Pull Requests

- Reference any related issues in the PR description.
- Ensure all linting checks pass.
- Request a review from at least one maintainer.
- Update [CHANGELOG.md](CHANGELOG.md) under the `[Unreleased]` section with a summary of your changes.

## License

By contributing, you agree that your contributions will be licensed under the project's [MIT License](LICENSE).
