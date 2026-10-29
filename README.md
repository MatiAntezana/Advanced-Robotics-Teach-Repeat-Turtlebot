# TP1 — Sistema Teach & Repeat para Robot Móvil Diferencial

Sistema integral de Teach & Repeat (T&R) implementado en ROS 2 Jazzy para plataformas móviles de tracción diferencial. El sistema permite registrar una trayectoria guiada por un operador humano y posteriormente reproducirla de manera autónoma utilizando localización por encoders en las ruedas, escaneo LiDAR 2D con ICP (Iterative Closest Point), o una fusión mixta de ambos sensores.

El proyecto está diseñado y validado tanto en entorno de simulación (Gazebo Harmonic con TurtleBot3 Burger) como en plataforma física de laboratorio (TurtleBot 4 sobre red local y Discovery Server FastDDS).

---

## 1. Requisitos y Entorno de Ejecución

El proyecto gestiona dependencias y entornos aislados mediante [Pixi](https://pixi.sh/) con el repositorio RoboStack para ROS 2 Jazzy y Python 3.12.

### Compilación y Verificación

Desde la raíz del repositorio (`Advance-Robotics-tp1`), ejecutar:

```bash
pixi run --frozen build
pixi run --frozen test
```

- La compilación genera el overlay en `ros_ws/install/` y enlaza el entorno con las dependencias de Gazebo Harmonic, Open3D, PyVista y herramientas de ROS 2.
- La suite de pruebas unitarias y de integración (`pixi run --frozen test`) ejecuta 26 tests verificando cinemática diferencial $SE(2)$, convergencia del controlador en coordenadas polares, consistencia del árbol TF, y robustez del alineamiento ICP.

---

## 2. Modos de Localización

El sistema implementa un estimador unificado de localización configurado mediante el argumento `mode`:

| Modo | Argumento | Principio de Funcionamiento |
|---|---|---|
| **Encoders** | `mode:=encoders` | Integración odometría diferencial pura a partir de las lecturas de velocidad/posición de ruedas en `/joint_states`. Rápido y continuo, sujeto a deriva por deslizamiento acumulado en curvas. |
| **LiDAR 2D** | `mode:=lidar` | Localización geométrica basada puramente en nube de puntos láser (`/scan`) mediante correspondencia Scan-to-Scan e ICP Scan-to-Keyframe (Open3D). Inmune a deslizamiento de ruedas. |
| **Mixto** | `mode:=mixed` | Fusión predictiva: utiliza odometría de encoders para predicción cinemática de alta frecuencia y corrige periódicamente la pose absoluta con cada escaneo LiDAR procesado por ICP. |

---

## 3. Árbol de Transformadas (/tf) y Tópicos

El sistema respeta rigurosamente las convenciones de marcos de referencia exigidas por la consigna del trabajo práctico:

### Etapa de Aprendizaje (Teach)
- **Transformadas activas en `/tf`:**
  - `odom -> reference`: Pose absoluta del keyframe de referencia activo en el mapa respecto al origen de odometría.
  - `reference -> base_link`: Pose relativa del robot respecto al keyframe de referencia actual.
- **Tópico `/rel_odom` (`nav_msgs/Odometry`):** Publica la posición y orientación relativa respecto al último keyframe registrado.
- **Registro de ruta:** Se añaden keyframes cada vez que el robot se desplaza una distancia superior a `distance_threshold` (0.20 m) o gira un ángulo mayor a `angle_threshold` (0.25 rad).

### Etapa de Repetición Autónoma (Repeat)
- **Transformadas activas en `/tf`:**
  - `odom -> reference` y `reference -> base_link`: Mantiene la localización unificada continua.
  - `base_link -> map_current_keyframe`: Localización relativa del robot respecto al marco del keyframe activo del Teach.
  - `map_current_keyframe -> map_next_keyframe`: Desplazamiento relativo hacia el siguiente keyframe objetivo en la secuencia aprendida.
  - `base_link -> setpoint`: Vector instantáneo de persecución hacia el objetivo local del controlador.
- **Controlador de Navegación:** Lazo cerrado en coordenadas polares $(\rho, \alpha, \beta)$ que publica consignas de velocidad en `/cmd_vel` para converger suavemente hacia cada keyframe hasta alcanzar la meta final.

---

## 4. Ejecución en Simulación (Gazebo Harmonic)

El directorio `sim/` contiene scripts modulares para automatizar el ciclo completo de simulación en el laberinto hexagonal de Gazebo:

### Paso 1: Lanzar el Simulador
En una terminal (Panel 1):
```bash
pixi run --frozen sim
```

### Paso 2: Fase Teach (Enseñar la Ruta)
En una segunda terminal (Panel 2), ejecutar el script de Teach indicando el modo deseado:
```bash
./sim/teach.sh mixed
# Modos disponibles: encoders | lidar | mixed
```
*El script lanza automáticamente RViz2 y graba el rosbag correspondiente en `evidence/simulacion/teach_bag_<modo>`.*

En una tercera terminal (Panel 3), abrir el nodo de teleoperación:
```bash
pixi run --frozen teleop
```
Conducir el robot con las teclas `w/a/s/d` y detenerlo con `espacio`. Al finalizar la trayectoria deseada, presionar `q` para salir del teleop.

En una cuarta terminal (Panel 4), solicitar el guardado de la ruta al servicio de Teach:
```bash
pixi run --frozen ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}'
```
Esperar la confirmación `success: true` y detener el script de Teach con `Ctrl+C`. La ruta se guarda en `routes/sim_route_<modo>.json`.

### Paso 3: Fase Repeat (Repetición Autónoma)
Ejecutar el script de Repeat para el modo correspondiente:
```bash
./sim/repeat.sh mixed
# Modos disponibles: encoders | lidar | mixed
```
*El script resetea automáticamente la pose del robot en Gazebo al origen exacto del Teach (`x=-2.0`, `y=-0.5`, `yaw=0.0`), lanza el controlador autónomo, abre RViz2 y graba el rosbag de validación en `evidence/simulacion/repeat_bag_<modo>`.*

Al alcanzar la meta final, el robot se detiene, publica `data: true` en `/tnr/done`, y el rosbag se cierra con su `metadata.yaml` validado.

---

## 5. Ejecución en Robot Real (TurtleBot 4)

El directorio `real/` contiene los scripts optimizados para despliegue en la plataforma física de laboratorio.

### Configuración de Red y Robots
El sistema admite conexión automática a los robots del laboratorio:
- **Robot 1 (por defecto):** IP `172.23.16.70`, namespace `/tb4_1`, `ROS_DOMAIN_ID=1`.
- **Robot 2:** IP `172.23.16.88`, namespace `/tb4_2`, `ROS_DOMAIN_ID=2`.

La sincronización de DDS se realiza mediante FastDDS Discovery Server apuntando al puerto `11811` del robot seleccionado.

### Comandos de Ejecución

1. **Enseñar ruta física (Teach):**
   ```bash
   ./real/teach.sh mixed 1
   # Argumentos opcionales: [modo] [robot_id] (por defecto: mixed 1)
   ```

2. **Teleoperar el robot físico:**
   ```bash
   ./real/teleop.sh 1
   ```

3. **Guardar la ruta física:**
   ```bash
   pixi run --frozen ros2 service call /tb4_1/tnr/save_route std_srvs/srv/Trigger '{}'
   ```

4. **Visualizar en RViz2:**
   ```bash
   ./real/rviz.sh 1
   ```

5. **Reproducir ruta autónoma (Repeat):**
   ```bash
   ./real/repeat.sh mixed 1
   ```

---

## 6. Visualización en RViz2

El archivo de configuración [`tp1_view.rviz`](ros_ws/src/Grupo4/tp1_teach_repeat/config/tp1_view.rviz) despliega:
- **Indicador de Método Activo:** Marcador de texto con el modo en ejecución (`ENCODERS`, `LIDAR`, `MIXTO`).
- **Trayectorias:** Línea continua del recorrido Teach (naranja) y trayectoria en vivo de Repeat (turquesa).
- **Keyframes:** Esferas y flechas de orientación sobre cada punto de referencia guardado.
- **Setpoint y Meta:** Vector visual hacia el objetivo instantáneo magenta y etiqueta de estado de aproximación.
- **Mapa de Celdas y Láser:** Nube de puntos proyectada y celdas de ocupación 2D correspondientes a las paredes detectadas.
- **Calidad de Alineación ICP:** Muestra en tiempo real el valor de RMSE residual en centímetros y el porcentaje de solapamiento (Overlap) del escaneo láser contra el mapa de keyframes.

---

## 7. Análisis de Resultados y Comparativa de Métricas

El proyecto cuenta con herramientas para analizar y contrastar el desempeño de los tres métodos de localización a partir de los rosbags grabados.

### Comparativa de Simulación (Gazebo)
Para procesar los rosbags de simulación (`evidence/simulacion/`) y calcular métricas contra el recorrido Teach:

```bash
pixi run python scripts/plot_sim_comparison.py
```

El script genera la figura integral [`evidence/simulacion/comparativa_simulacion.png`](evidence/simulacion/comparativa_simulacion.png) con 4 paneles de análisis:
1. **Trayectorias 2D Superpuestas:** Recorrido enseñado con sus 50 keyframes frente a la repetición autónoma en los 3 modos.
2. **Error Lateral en el Tiempo (Cross-Track Error):** Desvío ortogonal instantáneo respecto a la trayectoria del Teach.
3. **Perfil de Velocidades Comandadas:** Comportamiento del controlador en `/cmd_vel` ($v$ lineal y $\omega$ angular).
4. **Calidad de Alineación ICP:** RMSE residual de correspondencia geométrica en modos LiDAR y Mixto.

#### Tabla Resumen de Métricas (Simulación)
| Métrica | Encoders | LiDAR 2D (ICP) | Mixto |
|---|---|---|---|
| **Duración total** | 109.9 s | 114.2 s | 113.5 s |
| **Error lateral medio (CTE)** | 0.12 cm | 0.50 cm | 0.40 cm |
| **Error lateral máximo** | 1.30 cm | 2.13 cm | 2.00 cm |
| **Error lateral Percentil 95** | 0.42 cm | 1.30 cm | 1.12 cm |
| **Precisión de llegada a meta** | 2.75 cm | 2.63 cm | 3.03 cm |
| **Keyframes completados** | 100% (50/50) | 100% (52/52) | 100% (49/49) |

### Comparativa de Ensayos en Robot Real
Para procesar las bolsas de datos registradas en laboratorio (`evidence/laboratorio/`):

```bash
pixi run python scripts/plot_real_comparison.py
```

### Gráficos Individuales de Recorrido
Para generar visualizaciones detalladas de escaneos y keyframes por sesión:
```bash
python3 scripts/plot_run.py --bag evidence/simulacion/repeat_bag_mixed --route routes/sim_route_mixed.json --out evidence/sim_repeat_mixed.png
```

---

## 8. Estructura del Repositorio

```text
Advance-Robotics-tp1/
├── ros_ws/src/Grupo4/tp1_teach_repeat/   # Paquete principal ROS 2
│   ├── config/                           # Configuraciones de parámetros y vista RViz
│   ├── launch/                           # teach.launch.py y repeat.launch.py
│   ├── tp1_teach_repeat/                 # Nodos de localización, control y mapeo
│   └── test/                             # Suite de pruebas unitarias pytest
├── sim/                                  # Scripts de automatización para Gazebo
│   ├── common.sh                         # Rutas, tópicos y reseteo de pose
│   ├── teach.sh                          # Lanzamiento de Teach en simulación
│   └── repeat.sh                         # Lanzamiento de Repeat en simulación
├── real/                                 # Scripts para despliegue en TurtleBot 4 real
│   ├── common.sh                         # Configuración de IPs, puertos y Discovery Server
│   ├── teach.sh                          # Teach físico con rosbag
│   ├── repeat.sh                         # Repeat autónomo físico con rosbag
│   ├── teleop.sh                         # Teleoperación de robot real
│   └── rviz.sh                           # RViz2 conectado al robot real
├── routes/                               # Rutas guardadas en JSON (simulación y real)
├── evidence/
│   ├── simulacion/                       # Rosbags (.mcap) y gráficos de simulación
│   └── laboratorio/                      # Rosbags (.db3) y métricas de ensayos reales
├── scripts/                              # Scripts de análisis y generación de figuras
├── pixi.toml                             # Configuración del entorno reproducible Pixi
└── TP1.pdf                               # Consigna oficial de la cátedra
```
