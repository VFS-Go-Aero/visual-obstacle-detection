from glob import glob

from setuptools import find_packages, setup

package_name = 'startup_launcher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # `ros2 run startup_launcher stop` finds executables in lib/<package>
        ('lib/' + package_name, ['scripts/stop']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='vfspurdue@gmail.com',
    description='VFS drone stack bringup: ground and flight launch files',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wait_for_topic = startup_launcher.wait_for_topic:main',
            'camera_marker = startup_launcher.camera_marker:main',
        ],
    },
)
