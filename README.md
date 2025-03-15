# Тестовый Робот

ROS2 пакет для навигации мобильной робототехнической платформы на базе ховерборда.

## Описание

Этот пакет обеспечивает автономную навигацию робота, используя:
- Моторы от ховерборда в качестве привода (через hoverboard_driver_usart2)
- Камеру RealSense D435 для восприятия окружающей среды
- Nav2 для автономной навигации
- Преобразование глубины в лазерный скан для навигации

## Требования к системе

- Ubuntu 22.04
- ROS2 Humble
- Доступ к USB-портам для RealSense D435 и ховерборда

## Зависимости

Все зависимости указаны в package.xml:
- nav2_bringup
- realsense2_camera
- depthimage_to_laserscan
- nav2_map_server
- nav2_amcl
- nav2_lifecycle_manager
- tf2_ros
- rviz2
- hoverboard_driver_usart2

## Установка

1. Установите ROS2 Humble:
```bash
sudo apt update
sudo apt install -y ros-humble-desktop
```

2. Установите зависимости:
```bash
sudo apt install -y \
    ros-humble-nav2* \
    ros-humble-realsense2-camera \
    ros-humble-depthimage-to-laserscan
```

3. Клонируйте репозиторий:
```bash
cd ~/ros2_ws/src
git clone https://github.com/hdw0/test_robot.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Использование

1. Запустите основной launch-файл:
```bash
ros2 launch test_robot robot_launch.py
```

2. В RViz2:
   - Установите начальную позицию (2D Pose Estimate)
   - Задайте целевую точку (2D Goal Pose)
   - Используйте готовую карту или создайте новую

3. Мониторинг:
```bash
# Проверка топиков
ros2 topic list

# Просмотр одометрии
ros2 topic echo /odom

# Просмотр данных лазерного скана
ros2 topic echo /scan
```

## Конфигурация

- `config/nav2_params.yaml`: параметры навигации
- `config/nav2.rviz`: настройки визуализации
- `maps/map.yaml`: параметры карты

## Структура проекта
```
test_robot/
├── config/                 # Конфигурационные файлы
│   ├── nav2_params.yaml   # Параметры навигации
│   └── nav2.rviz         # Конфигурация RViz2
├── launch/                # Launch файлы
│   └── robot_launch.py   # Основной launch файл
├── maps/                  # Карты для навигации
│   ├── map.pgm           # Карта в формате изображения
│   └── map.yaml          # Метаданные карты
├── test/                  # Тесты
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── CMakeLists.txt        # Файл сборки
├── package.xml           # Описание пакета
├── setup.py             # Установочный скрипт Python
└── README.md            # Документация
```

## Тестирование

Запуск тестов:
```bash
colcon test --packages-select test_robot
```

## Лицензия

MIT License. См. файл [LICENSE](LICENSE).
