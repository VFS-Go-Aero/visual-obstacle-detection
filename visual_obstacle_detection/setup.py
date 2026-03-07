from setuptools import setup

package_name = "visual_obstacle_detection"

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
    description="Visual Obstacle Detection package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "point_cloud = visual_obstacle_detection.point_cloud:main",
            "obstacle_detection = visual_obstacle_detection.obstacle_detection:main",
            "obstacle_detection_pc = visual_obstacle_detection.obstacle_detection_pc:main",
            "scan_to_mav_3D = visual_obstacle_detection.scan_to_mav_3D:main",
            "scan_to_mavlink_sid = visual_obstacle_detection.scan_to_mavlink_sid:main",
        ],
    },
    url="https://github.com/VFS-Go-Aero/visual-obstacle-detection",
)
