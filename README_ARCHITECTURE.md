# 🤖 Puzzlebot Final Challenge - Análisis de Arquitectura

**Fecha de Análisis:** 6 de Junio de 2026  
**Estado del Proyecto:** ⚠️ NO OPERACIONAL - 3 bloqueadores P0  
**Prioridad:** CRÍTICA - El proyecto no compila ni corre actualmente

> **CAMBIO IMPORTANTE:** Este documento fue reevaluado tras comparar con proyecto funcional de referencia.
> **Resultado:** El README diagnosticaba bien la deuda técnica, pero se equivocaba en arquitectura y prioridades.
> La **Opción 1 (cadena de nodos)** es la correcta y es la que implementaremos.

---

## � BLOQUEADORES P0 — El Proyecto No Corre Sin Esto

El sistema **no compila ni arrancan los nodos** por tres razones encontradas tras análisis detallado:

### **P0.1: `bug2_FC_node.py` tiene error de sintaxis (IndentationError)**

**Ubicación:** Líneas 323–327  
**Problema:** El `elif` tiene el cuerpo en la misma línea

```python
elif returned_to_hit:self.change_state('STOP')          # ❌ cuerpo aquí
    self.get_logger().warn('Regrese al punto...')        # ❌ IndentationError
```

**Impacto:** `python3 -m py_compile bug2_FC_node.py` falla. El nodo **nunca se ejecuta**.

**Fix:** Reescribir con indentación normal:
```python
elif returned_to_hit:
    self.change_state('STOP')
    self.get_logger().warn(
        'Regrese al punto de impacto sin encontrar una linea M mejor. '
        'La meta puede estar bloqueada.'
    )
    self.cmd_pub.publish(Twist())
    self.goal_received = False
    return
```

---

### **P0.2: Los nodos NO se instalan (build roto)**

**Problema:** `package.xml` usa `ament_cmake` pero `CMakeLists.txt` solo instala scripts de `scripts/`:

```cmake
install(PROGRAMS
  scripts/puzzlebot_localization.py
  scripts/puzzlebot_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME})
```

Los **nodos reales** (`bug2_FC_node`, `ekf_physical_node`, `waypoint_manager`, `arucostatus`) están en **`scripts_JP/`** y NO se instalan.

**Impacto:** `executable='bug2_FC_node'` en launch falla con *"executable not found"*.

**Fix:** Convertir `puzzlebot_final` a `ament_python` + crear `setup.py` con entry_points:

```python
entry_points={
    'console_scripts': [
        'localisation_node = puzzlebot_final.localisation_node:main',
        'ekf_node          = puzzlebot_final.ekf_node:main',
        'bug2_node         = puzzlebot_final.bug2_node:main',
        'waypoint_manager  = puzzlebot_final.waypoint_manager:main',
        'aruco_monitor     = puzzlebot_final.aruco_monitor:main',
    ],
},
```

Esto unifica `scripts/` + `scripts_JP/` en un módulo Python importable con binarios registrados.

---

### **P0.3: El launch apunta a paquete inexistente y URDF roto**

**Problema en `Final_Challenge.launch.py`:**

```python
package_name = 'puzzlebot_sim'                  # ❌ no existe en este repo
urdf_file = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')  # ❌ solo hay .xacro
```

- `puzzlebot_sim` es resto de milestones de simulación (mc3/mc4/mc5). **El paquete real es `puzzlebot_final`**.
- Solo existe `puzzlebot.urdf.xacro`, no `puzzlebot.urdf` → `robot_state_publisher` revienta.

**Impacto:** Launch no resuelve dependencias; archivos URDF no encontrados.

**Fix:** 
1. Cambiar `package_name = 'puzzlebot_final'`
2. Cargar xacro correctamente (ver §2.2 abajo)
3. Sacar `robot_state_publisher` del launch de navegación; va en el bringup de hardware

---

**CONCLUSIÓN:** Antes de optimizar QoS, frecuencias o covarianzas, hay que conseguir que **el paquete construya, instale y arranque**. Los próximos pasos empiezan por **Sprint 0**.

---

## �📋 Tabla de Contenidos

1. [Visión General](#visión-general)
2. [Arquitectura de Nodos](#arquitectura-de-nodos)
3. [Flujo de Datos](#flujo-de-datos)
4. [Problemas Identificados](#problemas-identificados)
5. [Plan de Refactorización](#plan-de-refactorización)
6. [Matriz de Intercomunicación](#matriz-de-intercomunicación)
7. [Recomendaciones Inmediatas](#recomendaciones-inmediatas)

---

## 🎯 Visión General

El proyecto implementa un **robot navegador autónomo** que debe escapar de un laberinto usando:

| Componente | Tecnología | Función |
|------------|-----------|---------|
| **Localización** | EKF (Extended Kalman Filter) | Fusión de encoders + visión ArUco |
| **Percepción** | LiDAR RPLidar | Detección de obstáculos en tiempo real |
| **Visión** | ArUco markers | Correcciones de pose mediante marcadores |
| **Navegación** | Algoritmo Bug2 | Navegación reactiva sin mapas |
| **Control** | Control proporcional | Seguimiento de velocidades |
| **Hardware** | Puzzlebot (diferencial) | Robot de 2 ruedas con sensores |

**Objetivo Principal:** Llegar al waypoint final `(2.70, -0.60)` evitando obstáculos.

---

## 🏗️ Arquitectura de Nodos — OPCIÓN 1 (Cadena de Nodos)

> **CORRECCIÓN vs README anterior:** La arquitectura correcta es una **cadena de dos nodos de localización**,
> no un nodo monolítico. Este patrón es probado en la referencia y separable para debug.

### Disposición Física: Dos Capas Claramente Separadas

```
┌─────────────────────────────────────────────────────────┐
│                    HARDWARE FÍSICO                      │
│  Motor | Encoders | LiDAR | Cámara | Micro-ROS Bridge │
└───┬──────┬─────────┬──────┬────────┬───────────────────┘
    │      │         │      │        │

═══════════════════════════════════════════════════════════
                  CAPA A: HARDWARE BRINGUP
                 (puzzlebot_aruco.launch.xml)
═══════════════════════════════════════════════════════════

    │      │         │      │        │
    ├──────┤         │      │        │
    │      │         │      │        │
    ▼      ▼         ▼      ▼        ▼
┌────────────────────────────────────────────────────────┐
│  micro_ros_agent | rplidar_node | aruco_tracker       │
│  robot_state_publisher (URDF) | joint_state_publisher │
│  laser_tf_fix (TF estático)                            │
└────┬────────────────────────┬────────────────┬─────────┘
     │                        │                │
     ├─→ /VelocityEncR        ├─→ /scan        └─→ /marker_publisher/markers
     ├─→ /VelocityEncL        │
     └─→ /tf                  └─→ /tf_static

═══════════════════════════════════════════════════════════
           CAPA B: NAVEGACIÓN (final_challenge.launch.py)
═══════════════════════════════════════════════════════════

/VelocityEncR ─────┐
                   ▼
/VelocityEncL ──► localisation_node ──────────┐
                   (Odom crudo, dead-reckoning)│
                                              │
/marker_publisher/markers ──────────┐         │
                                   ▼         ▼
                              ekf_node (remap odom_raw)
                              (Corrección ArUco + Fusión)
                                   │
                                   ▼
                              /odom_ekf ◄─ ÚNICA FUENTE DE VERDAD
                                   │
           ┌───────────────────────┼───────────────────┐
           │                       │                   │
           ▼                       ▼                   ▼
     waypoint_manager          bug2_node          (rviz, opcional)
     (Genera /goal)         (Lee /odom_ekf)      (Visualización)
           │                  + /scan + /goal
           │                       │
           └──────────┬────────────┘
                      ▼
                   /cmd_vel ──► micro_ros_agent ──► motores
```

**Diferencia clave con intento anterior:**
- **Anterior (equivocado):** Un nodo EKF monolítico que hacía dead-reckoning + corrección + TF, además competía con otro nodo por los encoders.
- **Ahora (correcto):** Dos nodos encadenados:
  1. `localisation_node`: Solo dead-reckoning (simple, predecible)
  2. `ekf_node`: Solo corrección sobre el odom crudo (separable, debuggeable)
- **Ventaja:** En RViz comparas crudo vs fusionado. Es el patrón probado en la referencia.

### Tabla de Nodos: OPCIÓN 1 (Cadena)

| Nodo | Paquete | Responsabilidad | Entrada | Salida | Freq. | Estado |
|------|---------|-----------------|---------|--------|-------|--------|
| **CAPA A — Bringup Hardware** | | | | | | |
| `robot_state_publisher` | robot_state_publisher | Publica TF robot desde URDF xacro | URDF | `/tf` | ~100Hz | ✓ |
| `joint_state_publisher` | joint_state_publisher | Articulaciones (ruedas) | — | `/joint_states` | ~50Hz | ✓ |
| `rplidar_node` | rplidar_ros | Driver LiDAR | HW serial | `/scan` | 25Hz | ✓ |
| `aruco_tracker_autostart` | aruco_opencv | Detecta ArUco | `/image_raw` | `/marker_publisher/markers` | ~30Hz | ✓ |
| `micro_ros_agent` | (firmware bridge) | Encoders + motor control | HW → USB | `/VelocityEncR/L`, recibe `/cmd_vel` | ~50Hz | ✓ |
| `laser_tf_fix` | tf2_ros | TF estático láser | — | `/tf_static` | — | ✓ |
| **CAPA B — Navegación** | | | | | | |
| `localisation_node` | puzzlebot_final | **Odom crudo** (predicción pura) | `/VelocityEncR/L` | `/odom` | 20Hz | ✓ |
| `ekf_node` | puzzlebot_final | **Fusión EKF** (encoders + ArUco) | `/odom` (remapped a `odom_raw`), `/marker_publisher/markers` | `/odom_ekf` | 20Hz | ✓ |
| `bug2_node` | puzzlebot_final | **Navegación reactiva** (CONTROL PRINCIPAL) | `/odom_ekf`, `/scan`, `/goal` | `/cmd_vel` | 20Hz | ✓ |
| `waypoint_manager` | puzzlebot_final | **Generador de metas** | `/odom_ekf` | `/goal` | ~10Hz | ✓ |
| `aruco_monitor` | puzzlebot_final | Monitor ArUco (diagnóstico) | `/marker_publisher/markers` | logs | 1Hz | ✓ |
| `rviz2` | rviz2 | Visualizador 3D | `/tf`, `/scan`, `/odom_ekf` | — | UI | ✓ |

**Cambio clave en tabla vs README anterior:**
- Desaparece `ekf_physical_node` (nodo monolítico) → se divide en `localisation_node` + `ekf_node`
- Desaparece `raw_localisation_node` (redundante sin sentido)
- Un único publicador de odom que luego se transforma
- `bug2_node` lee `/odom_ekf`, no `/odom` directamente

---

## 📡 Flujo de Datos

### Detalles de Cada Flujo en Opción 1

**Las secciones específicas de flujo ya fueron refactorizadas arriba en la arquitectura.**

Lo importante a retener:
1. **Cadena de odom:** `localisation_node` → `odom_raw` → `ekf_node` → `odom_ekf`
2. **Consumidores únicos:** bug2_node y waypoint_manager leen `/odom_ekf`, no compiten
3. **Corrección visual independiente:** ArUco fluye en paralelo, solo el EKF lo usa
4. **Control centralizado:** bug2_node es el único que publica `/cmd_vel`
5. **Separabilidad:** Cambias localización sin tocar navegación

---

## ✅ Resumen de Cambios Respecto al README Original

| Punto | README Original | Realidad / Corrección |
|-------|-----------------|----------------------|
| Estado | "Operacional con deuda técnica" | ⚠️ NO operacional: 3 bloqueadores P0 |
| Problema #1 | "Saturación de encoders" | Parcialmente correcto: necesita arquit. cadena, no eliminación |
| Problema #2 | "Duplicación odom" | **INCORRECTA**: La duplicación es intencional en Opción 1 |
| Problema #3 | "Bug2 con funcs redundantes" | **FALSO**: Cada función es necesaria en patrón Bug2 |
| Fase 4 | "Combinar waypoint + bug2" | ❌ **NO HACER**: Referencia los mantiene separados |
| Fase 1.1 | "Comentar localisation_node" | ⚠️ **Reinterpretar**: No borrar, es parte de la cadena |
| Acción 4 | "Init covarianza con np.eye()*0.01" | ✓ **Correcto**: Se mantiene |

> **Conclusión:** El análisis original fue útil para identificar *deuda técnica de mantenibilidad*, pero la priorización estaba invertida. Antes de optimizar, hay que conseguir que **el código compile y corra**.

---

## 📋 Plan de Refactorización — SPRINTS CORREGIDOS

> **CAMBIO CRÍTICO:** La anterior "Fase 4" de combinar waypoint + bug2 estaba **EQUIVOCADA**.
> La arquitectura correcta (Opción 1) los mantiene separados. Nuevo plan basado en Sprint 0→3.

---

### **SPRINT 0 — Conseguir que Arranque (Medio día) — P0 BLOQUEADORES**

**SIN ESTO NADA MÁS IMPORTA.** El proyecto no compila ni corre actualmente.

#### Tarea 0.1: Arreglar IndentationError de `bug2_FC_node.py`
**Archivo:** `scripts_JP/bug2_FC_node.py` líneas 323–327

```python
# ANTES (❌ INCORRECTO):
elif returned_to_hit:self.change_state('STOP')
    self.get_logger().warn('Regrese al punto...')

# DESPUÉS (✅ CORRECTO):
elif returned_to_hit:
    self.change_state('STOP')
    self.get_logger().warn(
        'Regrese al punto de impacto sin encontrar una linea M mejor. '
        'La meta puede estar bloqueada.'
    )
    self.cmd_pub.publish(Twist())
    self.goal_received = False
    return
```

**Tiempo:** 5 minutos  
**Verificar:** `python3 -m py_compile scripts_JP/bug2_FC_node.py`

---

#### Tarea 0.2: Convertir `puzzlebot_final` a `ament_python`
**Cambios:**

1. **Reemplazar `package.xml`** (quitar `ament_cmake`, agregar `ament_python`):
```xml
<buildtool_depend>ament_python</buildtool_depend>
<exec_depend>rclpy</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>nav_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>tf2_ros</exec_depend>
<exec_depend>aruco_msgs</exec_depend>
<exec_depend>visualization_msgs</exec_depend>
```

2. **Crear `setup.py`**:
```python
from setuptools import setup, find_packages
import os

package_name = 'puzzlebot_final'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            os.listdir('launch')),
        (os.path.join('share', package_name, 'config'), 
            os.listdir('config')) if os.path.exists('config') else [],
        (os.path.join('share', package_name, 'rviz'), 
            os.listdir('rviz')) if os.path.exists('rviz') else [],
    ],
    install_requires=['setuptools', 'rclpy'],
    entry_points={
        'console_scripts': [
            'localisation_node = puzzlebot_final.localisation_node:main',
            'ekf_node          = puzzlebot_final.ekf_node:main',
            'bug2_node         = puzzlebot_final.bug2_node:main',
            'waypoint_manager  = puzzlebot_final.waypoint_manager:main',
            'aruco_monitor     = puzzlebot_final.aruco_monitor:main',
        ],
    },
)
```

3. **Crear `setup.cfg`**:
```ini
[develop]
script_dir=$base/lib/puzzlebot_final
```

4. **BORRAR `CMakeLists.txt`** (ya no se necesita)

**Tiempo:** 15 minutos

---

#### Tarea 0.3: Reorganizar código fuente en módulo Python

```bash
# Crear directorio del módulo
mkdir -p puzzlebot_final/puzzlebot_final

# Mover y renombrar scripts
mv scripts_JP/ekf_physical_node.py puzzlebot_final/puzzlebot_final/ekf_node.py
mv scripts_JP/bug2_FC_node.py puzzlebot_final/puzzlebot_final/bug2_node.py
mv scripts_JP/waypoint_manager.py puzzlebot_final/puzzlebot_final/waypoint_manager.py
mv scripts_JP/arucostatus.py puzzlebot_final/puzzlebot_final/aruco_monitor.py
mv scripts/puzzlebot_localization.py puzzlebot_final/puzzlebot_final/localisation_node.py

# Crear __init__.py
touch puzzlebot_final/puzzlebot_final/__init__.py

# Crear resource
mkdir -p puzzlebot_final/resource
echo puzzlebot_final > puzzlebot_final/resource/puzzlebot_final
```

**Tiempo:** 10 minutos

---

#### Tarea 0.4: Corregir `Final_Challenge.launch.py`

**Cambios:**

1. Línea 10: `package_name = 'puzzlebot_final'` (era `puzzlebot_sim`)
2. Eliminar la carga del URDF roto (líneas 42-49):
```python
# BORRAR estas líneas:
# with open(urdf_file, 'r') as infp:
#     robot_desc = infp.read()
# robot_state_publisher_node = Node(...)
```

3. Sacar `robot_state_publisher` del launch (va en bringup de hardware)

4. Cambiar remappings del EKF para la cadena correcta:
```python
localisation = Node(
    package=package_name,
    executable='localisation_node',
    name='localisation_node',
    output='screen',
    parameters=[{'use_sim_time': False}],
    remappings=[
        ('VelocityEncR', wr_topic),
        ('VelocityEncL', wl_topic),
    ],
)

ekf = Node(
    package=package_name,
    executable='ekf_node',
    name='ekf_node',
    output='screen',
    parameters=[...],
    remappings=[
        ('odom', 'odom_raw'),  # ← recibe odom crudo de localisation_node
        ('detection_topic', '/marker_publisher/markers'),
    ],
)

bug2 = Node(
    # ...
    remappings=[
        ('odom', 'odom_ekf'),  # ← consume odom fusionado del EKF
        # ... resto igual
    ],
)
```

**Tiempo:** 20 minutos

---

**Total Sprint 0:** ~50 minutos

**Validación:**
```bash
cd ~/FinalChallenge/Real_Robot/src
colcon build --packages-select puzzlebot_final
source install/setup.bash
ros2 launch puzzlebot_final final_challenge.launch.py
```

Todos los nodos deben aparecer sin "executable not found".

---

### **SPRINT 1 — Una Sola Fuente de Verdad (1 día)**

Ahora que construye y arranca, unificar parámetros duplicados.

#### Tarea 1.1: Crear `config/robot_params.yaml`
```yaml
robot:
  wheel_radius: 0.045        # MEDIR del robot real
  wheel_separation: 0.17     # MEDIR del robot real (eje a eje)
  encoder_scale_r: 0.01      # calibrar
  encoder_scale_l: 0.01      # calibrar

ekf:
  use_vision_correction: true
  aruco_map:
    70:  {x: 1.84, y: -0.30}
    705: {x: 0.90, y: -1.20}
    706: {x: 2.39, y: -1.26}
    708: {x: 1.19, y: -1.21}
    703: {x: 1.23, y: -2.07}
    702: {x: 0.28, y: -1.82}
    75:  {x: 2.74, y: -2.40}
    701: {x: 2.84, y: 0.00}
```

**Tiempo:** 15 minutos

---

#### Tarea 1.2: Parametrizar `ekf_node.py` y `localisation_node.py`
```python
# En ambos nodos:
self.declare_parameter('wheel_radius', 0.045)
self.declare_parameter('wheel_separation', 0.17)
self.declare_parameter('encoder_scale_r', 0.01)
self.declare_parameter('encoder_scale_l', 0.01)

self.r = self.get_parameter('wheel_radius').value
self.l = self.get_parameter('wheel_separation').value
# ... etc
```

Cargar en launch:
```python
Node(
    package=package_name,
    executable='ekf_node',
    parameters=[
        os.path.join(
            get_package_share_directory(package_name),
            'config', 'robot_params.yaml'
        )
    ],
)
```

**Tiempo:** 30 minutos

---

#### Tarea 1.3: Parametrizar `waypoint_manager.py`
```python
self.declare_parameter('waypoints', [(2.70, -0.60)])
self.declare_parameter('goal_tolerance', 0.20)

self.waypoints = self.get_parameter('waypoints').value
self.goal_tolerance = self.get_parameter('goal_tolerance').value
```

**Tiempo:** 10 minutos

---

#### Tarea 1.4: Eliminar duplicación de QoS en `aruco_monitor.py`
Líneas 39–40: **Quitar una de las dos suscripciones idénticas** al mismo tópico.

**Tiempo:** 5 minutos

---

**Total Sprint 1:** ~60 minutos

**Validación:** `ros2 param list` en el nodo debe mostrar los nuevos parámetros.

---

### **SPRINT 2 — Mejora de Navegación (2–3 días)**

Ahora que todo está modular y parametrizado, mejorar el algoritmo.

#### Tarea 2.1: Adoptar wall-following robusto de referencia
**Archivo:** `bug2_node.py`

La referencia tiene suavizado de comandos y recuperación al perder pared que el tuyo no tiene. Portar métodos:
- `enter_wall_following()`
- `exit_wall_following()`
- `follow_wall_command()`
- Suavizado exponencial de `w` en wall-following

**Tiempo:** 1 hora

---

#### Tarea 2.2: Consolidar parámetros de Bug2 en YAML
```yaml
bug2:
  # Distancias
  front_stop_distance: 0.22
  avoidance_start_distance: 0.38
  wall_follow_start_distance: 0.28
  
  # Tolerancias
  goal_tolerance: 0.05
  wall_follow_goal_tolerance: 0.18
  goal_pass_margin: 0.02
  
  # Ganancias
  k_rho: 0.8
  k_alpha: 1.5
  v_max: 0.15
  w_max: 0.40
  
  # Wall-following
  wall_follow_kw: 0.3
  wall_kp: 1.0
```

Cargar en `Final_Challenge.launch.py`.

**Tiempo:** 30 minutos

---

#### Tarea 2.3: Validar correcciones ArUco con `aruco_monitor`
En RViz, ver en tiempo real:
- Qué marcadores detecta
- Cuáles están en el mapa
- Pose estimada vs pose con corrección

**Tiempo:** 1–2 horas (iterativo)

---

**Total Sprint 2:** ~3–4 horas

---

### **SPRINT 3 — Pulido (Opcional)**

Formalizaciones y seguridad.

#### Tarea 3.1: State Machine formal con Enum
```python
from enum import Enum

class Bug2State(Enum):
    WAITING = 0
    GO_TO_GOAL = 1
    WALL_FOLLOWING = 2
    STOP = 3
```

Reemplazar strings por enum (más type-safe).

**Tiempo:** 30 minutos

---

#### Tarea 3.2: Documentación en README.md
- Arquitectura Opción 1
- Cómo correr el stack
- Parámetros configurables
- Troubleshooting

**Tiempo:** 1 hora

---

**Total Sprint 3:** ~1.5 horas

---

## 📊 Matriz de Intercomunicación

### Tabla de Tópicos Activos

| Tópico | Productor | Consumidores | Freq. | QoS | Estado | Prioridad |
|--------|-----------|--------------|-------|-----|--------|-----------|
| `/odom` | ekf_physical_node | bug2_FC_node, waypoint_manager | 20Hz | Standard | ✓ Crítico | ALTA |
| `/cmd_vel` | bug2_FC_node | micro_ros_agent | 20Hz | Standard | ✓ Crítico | ALTA |
| `/goal` | waypoint_manager | bug2_FC_node | ~10Hz | Latched | ✓ Crítico | ALTA |
| `/scan` | rplidar_node | bug2_FC_node | 25Hz | Sensor | ✓ Crítico | ALTA |
| `/aruco_detections` | aruco_tracker | ekf_physical_node, arucostatus | ~30Hz | Sensor | ✓ Importante | MEDIA |
| `/VelocityEncR` | micro_ros_agent | ekf_physical_node, localisation_node* | ~50Hz | Sensor | ✓ Crítico | ALTA |
| `/VelocityEncL` | micro_ros_agent | ekf_physical_node, localisation_node* | ~50Hz | Sensor | ✓ Crítico | ALTA |
| `/joint_states` | joint_state_publisher | localisation_node*, robot_state_publisher | ~50Hz | Standard | ✓ Importante | MEDIA |
| `/tf` | robot_state_publisher, ekf_physical_node | rviz2, tf2 | ~100Hz | — | ✓ Visual | BAJA |
| `/odom_est` | localisation_node* | — (no usado) | 100Hz | Standard | ✗ Muerta | BAJA |

\* = Marcar para eliminación

### Diagrama de Dependencias

```
                    Hardware
                       │
         ┌─────────────┼─────────────┐
         │             │             │
      Motor         Encoders      LiDAR, Cámara
         │             │             │
    micro_ros_agent    │        rplidar, aruco_tracker
         │             │             │
         │      ┌──────┴──────┐      │
         │      │             │      │
         │   /VelocityEnc  /image_raw /scan
         │   R/L              │      │
         │      │             │      │
         │   EKF ◄────────────┘      │
         │   Fusion           ┌──────┘
         │      │             │
         │   /odom    /aruco_detections
         │      │             │
         ├──────┴──────┐      │
         │             │      │
      bug2_FC_node  waypoint_mgr
         │             arucostatus
         │
      /cmd_vel
         │
         └──→ Motor (salida)
```

---

## ✅ Recomendaciones Inmediatas

### **Acción 1: Crear estructura de directorios de config**

```bash
mkdir -p puzzlebot_final/config
touch puzzlebot_final/config/robot_params.yaml
touch puzzlebot_final/config/aruco_map.yaml
touch puzzlebot_final/config/control_params.yaml
```

**Tiempo:** 5 minutos

### **Acción 2: Comentar `localisation_node` en launch**

Editar: `launch_JP/Final_Challenge.launch.py`

```python
# Comentar línea ~85-93:
# raw_odom_node = Node(
#     package=package_name,
#     executable='localisation_node',
#     ...
# )

# Comentar en return LaunchDescription ~195:
# raw_odom_node,
```

**Tiempo:** 5 minutos  
**Impacto:** -10% CPU, elimina redundancia

### **Acción 3: Crear carpeta `archived/` para scripts muertos**

```bash
mkdir -p puzzlebot_final/scripts_JP/archived
mv puzzlebot_final/scripts_JP/control_node.py puzzlebot_final/scripts_JP/archived/
mv puzzlebot_final/scripts_JP/trajectory_node.py puzzlebot_final/scripts_JP/archived/
mv puzzlebot_final/scripts_JP/bug0_FC_node.py puzzlebot_final/scripts_JP/archived/
```

**Tiempo:** 5 minutos  
**Impacto:** Claridad, historial mantenido

### **Acción 4: Inicializar covarianza EKF**

Editar: `scripts_JP/ekf_physical_node.py` línea 37

```python
# ANTES:
self.sigma = np.zeros((3, 3))

# DESPUÉS:
self.sigma = np.eye(3) * 0.01  # Inicializar pequeño pero no-singular
```

**Tiempo:** 2 minutos  
**Impacto:** Mejor convergencia EKF

### **Acción 5: Documentar changelog**

Crear: `CHANGELOG.md`

```markdown
# Changelog

## [Refactoring en Progreso]

### Identificado (Junio 6, 2026)
- [x] Redundancia: localisation_node duplica ekf_physical_node
- [x] Parametrización dispersa: waypoints, mapa ArUco hardcodeados
- [x] Scripts muertos: control_node, trajectory_node, bug0_FC_node
- [ ] Sincronización: waypoint_manager y bug2 desincronizados

### Fase 1 - En Progreso
- [ ] Comentar localisation_node
- [ ] Archivar scripts muertos
- [ ] Crear estructura de config
```

**Tiempo:** 10 minutos  
**Impacto:** Trazabilidad

---

## 🚀 Próximos Pasos

### Corto Plazo (Hoy-Mañana)
1. ✅ Generar este README
2. ⏳ Comentar `localisation_node`
3. ⏳ Archivar scripts muertos
4. ⏳ Crear carpetas de config

### Mediano Plazo (Esta semana)
5. Crear archivos YAML de configuración
6. Parametrizar `waypoint_manager.py`
7. Parametrizar `ekf_physical_node.py`
8. Consolidar QoS

### Largo Plazo (Próximas semanas)
9. Sincronizar ciclos de control
10. Combinar waypoint + bug2
11. Implementar state machine formal
12. Agregar fault tolerance

---

## 📚 Referencias

- **ROS2 QoS Profiles:** https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
- **Extended Kalman Filter:** Standard robotics reference (Thrun, Burgard, Fox)
- **Bug Algorithm:** Lumelsky & Stepanov (1987)

---

## 👥 Notas de Equipo

Este documento fue generado como resultado del análisis de arquitectura del proyecto **Puzzlebot Final Challenge** realizado el **6 de Junio de 2026**, y fue **REEVALUADO** tras comparación con proyecto funcional de referencia.

**Estado Actual:** 
- ❌ El sistema **NO es operacional** (3 bloqueadores P0)
- ✅ La arquitectura está identificada (Opción 1: cadena de nodos)
- ✅ El plan está estructurado (4 sprints, ~0.5 días + 1 día + 2–3 días + 1–2 horas)

**Próxima Acción:** Ejecutar SPRINT 0 (conseguir que compile y corra)

---

**Documento vivo:** Se actualiza tras cada sprint completado. Versión actual = v2.0 (reevaluada, arquitectura correcta)

