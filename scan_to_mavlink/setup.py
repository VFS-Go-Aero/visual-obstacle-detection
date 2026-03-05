from setuptools import setup

package_name = "scan_to_mavlink"

setup(
    name=package_name,
    version="0.7.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vertical Flight Systems Purdue",
    maintainer_email="vfspurdue@gmail.com",
    maintainer_url="https://vfspurdue.com",
    description="Converts LaserScan data to MAVLink obstacle distance messages",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scan_to_mavlink_node = scan_to_mavlink.scan_to_mavlink_node:main",
        ],
    },
    url="https://github.com/VFS-Go-Aero/visual-obstacle-detection",
)
