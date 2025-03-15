#!/bin/bash

# Очистка переменных окружения
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

# Источник ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Сборка
colcon build