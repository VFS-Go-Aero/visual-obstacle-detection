from setuptools import setup
import os
from glob import glob

package_name = "launch_files"

setup(
    name=package_name,
    version="0.10.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vertical Flight Systems Purdue",
    maintainer_email="vfspurd@purdue.edu",
    description="Holds relevant launch files for the visual obstacle detection system",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drone_pose = launch_files.drone_pose:main",
        ],
    },
    url="https://github.com/VFS-Go-Aero/visual-obstacle-detection",
)
