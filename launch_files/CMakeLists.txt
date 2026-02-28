cmake_minimum_required(VERSION 3.5)
project(launch_files)  # must match package.xml <name>

# find ament_cmake to handle build and install
find_package(ament_cmake REQUIRED)

# install all launch files so ROS 2 can find them
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# mark this as a ROS 2 package
ament_package()