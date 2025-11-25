# Zephyr_project

**Frictionless robot thesis**

Este repositorio contiene el código y recursos asociados al proyecto de tesis sobre un robot sin fricción, desarrollado para demostrar control avanzado y manipulación en un entorno tanto simulado como real bajo plataformas ROS 2.

## Tabla de Contenidos

- [Descripción](#descripción)
- [Requisitos](#requisitos)
- [Instalación](#instalación)
- [Uso](#uso)
- [Estructura](#estructura)
- [Créditos](#créditos)

## Descripción

Zephyr_project utiliza ROS 2 y está principalmente implementado en Python y C++. El objetivo es proporcionar un entorno de pruebas para el control de un robot sin fricción, facilitando experimentación con temas avanzados de robótica.

## Requisitos

- ROS 2 Humble Hawksbill (o versión compatible)
- Python 3.x
- Compilador de C++

## Instalación

1. Clona el repositorio e instala dependencias (asumiendo un workspace de ROS 2):

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Cnoobmaster69/Zephyr_project.git
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build
   source install/setup.bash
   ```

## Uso

1. Inicia el entorno de simulación y el modelo básico del robot:

   ```bash
   ros2 launch jaeger_model basic_jaeger.launch.py
   ```

2. Publica un setpoint de posición al robot en una terminal diferente:

   ```bash
   ros2 topic pub /position_setpoint geometry_msgs/msg/Point "{x: -0.3, y: -0.3, z: 0.0}"
   ```

   Puedes modificar los valores de X y Y para experimentar con diferentes posiciones.

## Estructura

- `jaeger_model/`: Paquete de plugins de ros2_control
- `imu_bmp_pub/`: Paquete publicador de IMU, encendido de motores y teleop
- `CMakeLists.txt`, `package.xml`: Configuración de ROS 2 y CMake
- `README.md`: Este archivo

## Créditos

Desarrollado por [Cnoobmaster69](https://github.com/Cnoobmaster69) como parte de su tesis de robot sin fricción.

---

¡Contribuciones y sugerencias son bienvenidas!

