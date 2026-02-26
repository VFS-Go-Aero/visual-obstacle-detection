from setuptools import find_packages, setup

package_name = 'scan_to_mavlink'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vertical Flight Systems Purdue',
    maintainer_email='vfspurdue@gmail.com',
    description='Converts LaserScan data to MAVLink obstacle distance messages',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'scan_to_mavlink_node = scan_to_mavlink.scan_to_mavlink_node:main',
        ],
    },
)
