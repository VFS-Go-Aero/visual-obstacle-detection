from setuptools import setup

package_name = "visual_obstacle_detection"

setup(
    name=package_name,
    version="0.4.3",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vertical Flight Systems Purdue",
    maintainer_email="vfspurdue@gmail.com",
    description="Visual Obstacle Detection package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "point_cloud = visual_obstacle_detection.point_cloud:main",
            "publisher = visual_obstacle_detection.publisher:main",
        ],
    },
)
