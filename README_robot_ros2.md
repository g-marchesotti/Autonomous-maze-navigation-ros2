# 🤖 Autonomous Maze Navigation — ROS2 + Nav2 + SLAM

Robot diferencial autónomo con navegación a punto objetivo, mapeo del entorno (SLAM) y evasión de obstáculos en tiempo real, simulado en Gazebo sobre ROS2 Jazzy.

## Demo

> 📹 [Video de funcionamiento](REEMPLAZAR_CON_LINK)

---

## Descripción

El sistema permite indicarle al robot un punto de destino en el mapa. A partir de ese punto, planifica una ruta óptima, navega de forma autónoma y reacciona en tiempo real a los obstáculos detectados por su sensor LIDAR, recalculando el camino si es necesario.

El flujo completo incluye tres etapas: primero el robot recorre el entorno usando un algoritmo de seguimiento de paredes para construir el mapa (SLAM). Una vez guardado el mapa, se lanza el stack de navegación Nav2 que permite enviarle objetivos al robot desde RViz2.

---

## Stack tecnológico

| Tecnología | Rol |
|---|---|
| ROS2 Jazzy | Framework de robótica |
| Gazebo | Simulación del robot y entorno |
| Nav2 | Planificación de rutas y control |
| SLAM Toolbox | Mapeo y localización simultáneos |
| LIDAR (simulado) | Detección de obstáculos |
| RViz2 | Visualización y envío de objetivos |

---

## Arquitectura del sistema

```
┌─────────────────────────────────────────────┐
│                 Gazebo (simulación)          │
│   Robot diff_drive + LIDAR simulado          │
└────────────────┬────────────────────────────┘
                 │ /scan, /odom, /cmd_vel
┌────────────────▼────────────────────────────┐
│                  ROS2                        │
│                                             │
│  ┌─────────────┐    ┌──────────────────┐    │
│  │ SLAM Toolbox│    │      Nav2        │    │
│  │  (mapping)  │    │ Planner + Control│    │
│  └──────┬──────┘    └────────┬─────────┘    │
│         │ /map               │ /cmd_vel      │
│         └────────────────────┘              │
└─────────────────────────────────────────────┘
```

**Flujo de navegación:**
1. El nodo `seguidor_paredes` recorre el laberinto y SLAM construye el mapa
2. El mapa se guarda con `map_saver_cli`
3. Nav2 carga el mapa y el usuario indica un goal en RViz2
4. El planificador global calcula la ruta y el controlador local la ejecuta esquivando obstáculos

---

## Decisiones de diseño

**Seguidor de paredes para el mapeo:** en lugar de teleoperar el robot manualmente para mapear el laberinto, se implementó un nodo autónomo que sigue las paredes usando las lecturas del LIDAR. Esto permite construir el mapa de forma sistemática y repetible.

**Nav2 sobre el mapa pregenerado:** separar la etapa de mapeo de la de navegación simplifica el sistema — Nav2 trabaja sobre un mapa estático conocido, lo que hace la planificación más eficiente y predecible dentro del laberinto.

**Parámetros personalizados (`mis_params.yaml`):** los parámetros por defecto de Nav2 no estaban optimizados para el entorno del laberinto (espacios angostos, giros cerrados). Se ajustaron los tolerancias del controlador local y los radios de inflación del costmap.

---

## Cómo ejecutar

### Requisitos previos

```bash
# ROS2 Jazzy instalado
# Instalar dependencias
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-nav2*
```

### 1. Clonar y compilar

```bash
git clone https://github.com/TU_USUARIO/autonomous-maze-navigation-ros2.git
cd autonomous-maze-navigation-ros2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select control_lab
source install/local_setup.bash
```

### 2. Lanzar la simulación en Gazebo

```bash
# Terminal 1
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch ros_gz_example_bringup diff_drive.launch.py
```

### 3. Ejecutar el seguidor de paredes

```bash
# Terminal 2
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 run control_lab seguidor_paredes
```

### 4. Lanzar SLAM y construir el mapa

Antes de ejecutar, editar `/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml`:

```yaml
odom_frame: diff_drive/odom
map_frame: map
base_frame: diff_drive
scan_topic: /diff_drive/scan
use_map_saver: true
mode: mapping
use_sim_time: true
```

```bash
# Terminal 3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```

Una vez que el mapa esté completo, guardarlo:

```bash
# Terminal 4
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f mapa_laberinto
```

Cerrar todo una vez guardado el mapa.

### 5. Navegación autónoma con Nav2

```bash
# Terminal 1 — Gazebo (igual que paso 2, sin ejecutar el seguidor)

# Terminal 2 — Nav2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:=mapa_laberinto.yaml \
  params_file:=mis_params.yaml
```

### 6. Configuración de RViz2

En el panel izquierdo de RViz2:
- `fixed frame` → `map`
- `Add` → `By topic` → `/map`
- En el Map agregado: `Durability Policy` → `Transient Local`

Dar play en Gazebo, luego usar **Goal Pose** en RViz2 para indicar el destino. El robot navegará de forma autónoma.

---

## Estructura del repositorio

```
autonomous-maze-navigation-ros2/
├── src/
│   └── control_lab/
│       ├── control_lab/
│       │   └── seguidor_paredes.py    # Nodo de seguimiento de paredes
│       ├── launch/
│       └── package.xml
├── config/
│   └── mis_params.yaml                # Parámetros personalizados Nav2
├── maps/                              # Mapas pregenerados del laberinto
└── README.md
```

---

*Proyecto desarrollado en colaboración con [Nombre Compañero] para la materia [Nombre Materia], UNLP.*
