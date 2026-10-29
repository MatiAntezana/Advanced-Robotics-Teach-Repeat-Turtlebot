# TP1 — Sistema Teach & Repeat para Robot Móvil Diferencial

Autores: Matias Luciano Antezana, Ignacio Schuemer y Segundo Santos Torrado.

Trabajo práctico de Robótica Avanzada, Grupo 4. Implementa Teach & Repeat en ROS 2: se enseña una ruta por teleoperación y luego el robot la repite de forma autónoma siguiendo referencias guardadas (keyframes).

La localización puede usar encoders, LiDAR 2D con ICP de Open3D o una combinación de ambos. Se probó en Gazebo Harmonic con TurtleBot3 Burger y en el laboratorio con TurtleBot 4, base iRobot Create 3.

La [consigna](TP1.pdf) asigna 4 puntos al modo encoders, 4 a LiDAR y 2 al mixto.

## Instalación y compilación

El entorno usa [Pixi](https://pixi.sh/) y RoboStack, con ROS 2 Jazzy y Python 3.12 en macOS Apple Silicon (`osx-arm64`).

### Entorno

Desde la raíz del repositorio (`Advanced-Robotics-tp1`):

```bash
pixi install --frozen
```

`--frozen` usa las versiones fijadas en `pixi.lock`.

### Compilar

Para compilar el espacio de trabajo local:

```bash
pixi run --frozen build
```

La tarea ejecuta `colcon build` sin `--symlink-install` y copia los archivos a `ros_ws/install/`. Después de cambiar código, archivos launch o parámetros, hay que volver a compilar para que se apliquen los cambios.

### Pruebas

Para ejecutar las pruebas:

```bash
pixi run --frozen test
```

Las pruebas usan `ROS_DOMAIN_ID=43` para no interferir con los robots o las simulaciones en ejecución. Los 26 casos se distribuyen así:

| Archivo | Casos | Qué comprueba |
|---|---:|---|
| `test_core.py` | 7 | Cinemática diferencial, transformaciones SE(2), interpolación temporal, ICP, control polar y guardado de rutas. |
| `test_mapping.py` | 2 | Grilla de consenso y filtrado de puntos aislados. |
| `test_nodes.py` | 11 | Fuentes de localización, recuperación del láser, timestamps TF, publicación a 50 Hz, submapas y frenado ante mensajes obsoletos. |
| `test_runtime.py` | 2 | Exclusión de instancias duplicadas de Gazebo y cierre con Ctrl+C. |
| `test_visualization.py` | 4 | Etiquetas de keyframes, setpoint, métricas de alineación y timestamps de las nubes. |

### Parámetros

Hay dos archivos de parámetros:

- Simulación (`ros_ws/src/Grupo4/tp1_teach_repeat/config/params.yaml`): Calibrado para la dinámica del TurtleBot3 Burger en Gazebo Harmonic.

- Laboratorio Real (`params_tb4.yaml`): Ajustado a las dimensiones, articulaciones y límites de tracción del TurtleBot 4 (iRobot Create 3).

| Parámetro ROS | Archivo YAML | Valor Sim (TB3) | Valor Real (TB4) | Descripción |
|---|---|:---:|:---:|---|
| `wheel_radius` | Ambos | `0.033` m | `0.036` m | Radio efectivo de rueda motriz. |
| `wheel_separation` | Ambos | `0.160` m | `0.233` m | Distancia entre ruedas (trocha cinemática). |
| `left_joint` / `right_joint` | Ambos | `wheel_left_joint` / `wheel_right_joint` | `left_wheel_joint` / `right_wheel_joint` | Nombres de articulaciones suscritas en `/joint_states`. |
| `keyframe_distance` | Ambos | `0.20` m | `0.20` m | Umbral de traslación (Teach): distancia mínima para registrar un nuevo keyframe. |
| `keyframe_angle` | Ambos | `0.25` rad (~14.3°) | `0.25` rad (~14.3°) | Umbral de rotación (Teach): giro angular mínimo para registrar un nuevo keyframe. |
| `teach_submap_keyframes`| Ambos | `3` | `3` | Tamaño de ventana de keyframes previos para alineación local de mapa (mitiga deriva scan-to-scan). |
| `icp_distance` | Ambos | `0.25` m | `0.30` m | Distancia máxima de correspondencia de puntos en ICP. |
| `icp_fitness` / `icp_rmse`| Ambos | `0.55` / `0.10` m | `0.50` / `0.12` m | Umbrales de solapamiento mínimo y RMSE residual aceptable para validar la pose ICP. |
| `position_tolerance` | Ambos | `0.035` m (3.5 cm) | `0.040` m (4.0 cm) | Tolerancia de llegada al keyframe meta final. *(Para keyframes intermedios se expande a $\max(2 \times \text{tol}, 0.08\,\text{m})$ para marcha continua).* |
| `yaw_tolerance` | Ambos | `0.06` rad (~3.4°) | `0.08` rad (~4.6°) | Tolerancia de alineación angular en la meta final *(en waypoints intermedios el rumbo es libre).* |
| `max_v` / `max_w` | Ambos | `0.18` m/s / `1.0` rad/s | `0.30` m/s / `1.5` rad/s | Velocidad lineal y angular máximas del controlador polar (máxima velocidad física segura del TurtleBot 4). |
| `k_rho`, `k_alpha`, `k_beta`, `k_yaw` | Ambos | `0.9`, `2.5`, `-0.6`, `1.5` | `1.2`, `2.2`, `-0.6`, `1.5` | Ganancias de control polar en lazo cerrado (estabilidad Lyapunov con $k_\rho > 0$, $k_\beta < 0$, $k_\alpha > k_\rho$). |

## Modos de localización

El argumento `mode` selecciona la fuente de localización:

| Modo | Argumento | Principio de Funcionamiento |
|---|---|---|
| Encoders | `mode:=encoders` | Integración odometría diferencial pura a partir de las lecturas de velocidad/posición de ruedas en `/joint_states`. Rápido y continuo, sujeto a deriva por deslizamiento acumulado en giros. |
| LiDAR 2D | `mode:=lidar` | Localización geométrica basada puramente en nube de puntos láser (`/scan`) mediante correspondencia Scan-to-Scan e ICP Scan-to-Keyframe (Open3D). Inmune a deslizamiento de ruedas. |
| Mixto | `mode:=mixed` | Fusión predictiva: utiliza odometría de encoders para predicción cinemática continua de alta frecuencia y corrige periódicamente la pose absoluta con cada escaneo LiDAR procesado por ICP. |

## Transformaciones y tópicos

Las transformaciones se publican en `/tf`. Teach y Repeat comparten los marcos `odom`, `reference` y `base_link`.

### Teach

- Transformadas activas en `/tf`:
  - `odom -> reference`: Pose absoluta del keyframe de referencia activo en el mapa respecto al origen de odometría.
  - `reference -> base_link`: Pose relativa del robot respecto al keyframe de referencia actual.

- Tópico `/rel_odom` (`nav_msgs/Odometry`): Publica la posición y orientación relativa del robot respecto al último keyframe registrado.

- Registro de ruta: Se añaden keyframes cada vez que el robot se desplaza una distancia superior a `keyframe_distance` (0.20 m) o gira un ángulo mayor a `keyframe_angle` (0.25 rad).

### Repeat

- Transformadas activas en `/tf`:
  - `odom -> reference` y `reference -> base_link`: Mantiene la localización unificada continua.
  - `base_link -> map_current_keyframe`: Localización relativa del robot respecto al marco del keyframe activo del Teach.
  - `map_current_keyframe -> map_next_keyframe`: Desplazamiento relativo hacia el siguiente keyframe objetivo en la secuencia aprendida.
  - `base_link -> setpoint`: Vector instantáneo de persecución hacia el objetivo local del controlador.

- Controlador de Navegación: Lazo cerrado en coordenadas polares $(\rho, \alpha, \beta)$ que publica consignas de velocidad en `/cmd_vel` para converger suavemente hacia cada keyframe hasta alcanzar la meta final.

## Simulación

Los scripts de `sim/` reposicionan el robot, lanzan los nodos y graban el rosbag. Más abajo se muestran los comandos para ejecutar los componentes por separado.

### Enseñar y repetir una ruta

#### 1. Gazebo (terminal 1)

Inicia Gazebo Harmonic con el mundo del laberinto hexagonal (`turtlebot3_world`), el puente ROS-Gazebo (`ros_gz_bridge`), y el TurtleBot3 Burger posicionado en `x=-2.0`, `y=-0.5`:
```bash
pixi run --frozen sim
```

#### 2. Teach (terminal 2)

Inicia el nodo de aprendizaje, resetea la pose inicial en Gazebo y graba el rosbag en segundo plano:
```bash
./sim/teach.sh mixed
# Modos disponibles: encoders | lidar | mixed (por defecto: lidar)
```
Archivos generados:

- Ruta serializada: `routes/sim_route_<modo>.json`

- Rosbag: `evidence/simulacion/teach_bag_<modo>`

#### 3. Teleoperación (terminal 3)

En una tercera terminal, abre el nodo de teleoperación por teclado:
```bash
pixi run --frozen teleop
```

- Controles: `w` (avanzar), `s` (retroceder), `a` (girar izquierda), `d` (girar derecha), `espacio` (frenar).

- Presionar `q` para salir del teleoperador al terminar el recorrido guiado.

#### 4. Guardar la ruta

Existen dos formas de consolidar la ruta enseñada:

- Guardado Automático: En la terminal de Teach (Panel 2), presionar `Ctrl+C`. El script intercepta la señal mediante su trampa (`trap cleanup EXIT`), solicita automáticamente el guardado de la ruta al servicio `/tnr/save_route`, aguarda la sincronización del archivo `.json`, y cierra ordenadamente el rosbag garantizando un `metadata.yaml` íntegro.

- Guardado Manual con Verificación: En una terminal auxiliar (Panel 4), invocar el servicio ROS 2:
  ```bash
  pixi run --frozen ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}'
  ```
  Esperar la respuesta `success: true`. Luego, presionar `Ctrl+C` en el Panel 2 para cerrar el rosbag.

#### 5. Repeat (terminal 2)

Una vez guardada la ruta del modo correspondiente, ejecutar:
```bash
./sim/repeat.sh mixed
# Modos disponibles: encoders | lidar | mixed (por defecto: lidar)
```
Comportamiento del script:

1. Reposiciona automáticamente al TurtleBot3 en Gazebo al origen exacto del Teach (`x=-2.0, y=-0.5, yaw=0.0`).
2. Inicia la grabación del rosbag de validación en `evidence/simulacion/repeat_bag_<modo>`.
3. Lanza el nodo de localización y el controlador en lazo cerrado para reproducir los keyframes aprendidos.
4. Cuando el robot alcanza la meta, se detiene y publica `data: true` en `/tnr/done`.
5. Cierre: Al visualizar el fin de la trayectoria (`keyframe=N:done=True`), presionar `Ctrl+C` en la terminal para consolidar y cerrar el rosbag con su metadato validado.

### RViz

Los scripts de simulación ejecutan por defecto sin interfaz RViz2 para optimizar recursos. Para monitorear visualmente las trayectorias, nubes de puntos y keyframes en vivo, abrir otra terminal y ejecutar:
```bash
pixi run --frozen rviz
```
(Opcional: para inspeccionar exclusivamente el árbol de marcos coordenados TF, usar `pixi run --frozen rviz-tf`).

Para lanzar Repeat con RViz2 integrado directamente en el mismo proceso, invocar:
```bash
RVIZ=true ./sim/repeat.sh mixed
```

### Ejecutar sin los scripts

Si se desea prescindir de los scripts shell y ejecutar cada componente de forma nativa:

1. Lanzar Gazebo Harmonic sin GUI (modo headless):
   ```bash
   pixi run --frozen ros2 launch tp1_teach_repeat sim.launch.py gui:=false
   ```

2. Reposicionar manualmente el robot en Gazebo:
   ```bash
   pixi run --frozen gz service -s /world/default/set_pose \
     --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
     --req 'name: "burger" position: {x: -2.0 y: -0.5 z: 0.01} orientation: {w: 1.0}'
   ```

3. Lanzar Teach nativo:
   ```bash
   pixi run --frozen ros2 launch tp1_teach_repeat teach.launch.py \
     mode:=mixed route_file:=routes/sim_route_mixed.json overwrite:=true rviz:=true
   ```

4. Lanzar Repeat nativo:
   ```bash
   pixi run --frozen ros2 launch tp1_teach_repeat repeat.launch.py \
     mode:=mixed route_file:=routes/sim_route_mixed.json rviz:=true
   ```

## Robot real

Los scripts de `real/` permiten ejecutar el sistema en los TurtleBot 4 del laboratorio.

### Red del laboratorio

Para comunicarse con las plataformas, la estación de trabajo debe estar conectada a la misma red local (Wi-Fi/LAN) del laboratorio:

- Robot 2 (por defecto): IP `172.23.16.88`, namespace `/tb4_2`, `ROS_DOMAIN_ID=2`.

- Robot 1: IP `172.23.16.70`, namespace `/tb4_1`, `ROS_DOMAIN_ID=1`.

La sincronización DDS se realiza mediante FastDDS Discovery Server apuntando al puerto `11811` del robot seleccionado (`ROS_DISCOVERY_SERVER="<IP>:11811"`). Los scripts gestionan estas variables de entorno, perfiles super-client y namespaces de forma automática.

Para abrir una terminal interactiva con el entorno DDS cargado (útil para diagnósticos como `ros2 topic list` o `ros2 topic hz`):
```bash
source real/env.sh 2   # Carga entorno para Robot 2 (o 'source real/env.sh 1' para Robot 1)
```

### Enseñar y repetir una ruta

#### 1. Teach

En la primera terminal, iniciar el nodo Teach indicando el modo de localización (`encoders`, `lidar` o `mixed`), el robot (`2` o `1`) y opcionalmente el circuito (por defecto `circuito-2`):
```bash
./real/teach.sh lidar
# Sintaxis: ./real/teach.sh [encoders|lidar|mixed] [robot] [circuito]
# Por defecto: lidar 2 circuito-2
```
Archivos generados:
- Ruta serializada: `routes/<circuito>/lab_route_<modo>.json` (y copia en `routes/lab_route_<modo>.json`)
- Rosbag Teach: `evidence/laboratorio/<circuito>/teach_bag_<modo>`

#### 2. Teleoperación

En una segunda terminal, iniciar la teleoperación por teclado hacia el robot deseado:
```bash
./real/teleop.sh
# Sintaxis: ./real/teleop.sh [2|1] (por defecto: 2)
```

- Controles: `i` (avanzar), `,` (retroceder), `j` (girar izquierda), `l` (girar derecha), `k` (frenar).

- Conducir el robot registrando la ruta deseada. Al finalizar, salir del teleop con `Ctrl+C`.

#### 3. RViz

En una tercera terminal, visualizar las transformadas, el escaneo LiDAR y los keyframes en tiempo real:
```bash
./real/rviz.sh
# Sintaxis: ./real/rviz.sh [2|1] (por defecto: 2)
```

#### 4. Guardar la ruta

- Método automático (recomendado): Presionar `Ctrl+C` en la terminal donde corre `./real/teach.sh`. El script ejecuta el guardado automático de la ruta y cierra el rosbag de forma limpia.

- Método manual (desde otra terminal):
  ```bash
  source real/env.sh
  ros2 service call /tnr/save_route std_srvs/srv/Trigger '{}'
  ```

#### 5. Repeat

1. Colocar físicamente el robot en la posición y orientación inicial del Teach.
2. Ejecutar el script de repetición:
```bash
./real/repeat.sh lidar
# Sintaxis: ./real/repeat.sh [encoders|lidar|mixed] [robot] [circuito]
# Por defecto: lidar 2 circuito-2
# Opcional con RViz integrado: RVIZ=true ./real/repeat.sh lidar
```
El controlador polar guiará al robot a lo largo de los keyframes aprendidos y registrará las métricas de repetición en `evidence/laboratorio/<circuito>/repeat_bag_<modo>` hasta publicar `data: true` en `/tnr/done`.

## Qué muestra RViz

El archivo de configuración [`tp1_view.rviz`](ros_ws/src/Grupo4/tp1_teach_repeat/config/tp1_view.rviz) despliega:

- Indicador de Método Activo: Marcador de texto con el modo en ejecución (`ENCODERS`, `LIDAR`, `MIXTO`).

- Trayectorias: Línea continua del recorrido Teach (naranja) y trayectoria en vivo de Repeat (turquesa).

- Keyframes: Esferas y flechas de orientación sobre cada punto de referencia guardado.

- Setpoint y Meta: Vector visual hacia el objetivo instantáneo magenta y etiqueta de estado de aproximación.

- Mapa de Celdas y Láser: Nube de puntos proyectada y celdas de ocupación 2D correspondientes a las paredes detectadas.

- Calidad de Alineación ICP: Muestra en tiempo real el valor de RMSE residual en centímetros y el porcentaje de solapamiento (Overlap) del escaneo láser contra el mapa de keyframes.

## Resultados y gráficos

Los scripts de `scripts/` generan los gráficos y las métricas a partir de los rosbags de simulación y laboratorio.

### Simulación

Para procesar las bolsas de datos de simulación (`evidence/simulacion/`) y calcular las métricas respecto a la trayectoria del Teach:

```bash
pixi run python scripts/plot_sim_comparison.py
```
Argumentos opcionales:

- `--output`: Ruta de la imagen resultante (por defecto: `evidence/simulacion/comparativa_simulacion.png`).

- `--summary`: Ruta del archivo Markdown de resumen (por defecto: `evidence/simulacion/metricas_simulacion.md`).

El script procesa los rosbags en formato MCAP y genera:

1. Gráfico de tres paneles ([`comparativa_simulacion.png`](evidence/simulacion/comparativa_simulacion.png)):
   - Panel 1 (Izquierda): Trayectorias 2D superpuestas del recorrido enseñado (con sus keyframes, inicio y meta) frente a las repeticiones autónomas de Encoders, LiDAR 2D y Mixto, en proporción 1:1 (`axis('equal')`).
   - Panel 2 (Superior derecho): Error lateral instantáneo (Cross-Track Error en cm) a lo largo del tiempo de navegación, suavizado mediante ventana Hanning.
   - Panel 3 (Inferior derecho): Calidad de la correspondencia geométrica ICP LiDAR (RMSE residual scan-to-keyframe en cm) para los modos LiDAR y Mixto.
2. Resumen de métricas ([`metricas_simulacion.md`](evidence/simulacion/metricas_simulacion.md)):
   Actualiza automáticamente la tabla comparativa con duraciones, errores estadísticos y tasa de keyframes completados.

#### Resultados registrados

| Métrica | Encoders | LiDAR 2D (ICP) | Mixto |
|---|---|---|---|
| Duración total | 109.9 s | 114.2 s | 113.5 s |
| Error lateral medio (CTE) | 0.12 cm | 0.50 cm | 0.40 cm |
| Error lateral máximo | 1.30 cm | 2.13 cm | 2.00 cm |
| Error lateral Percentil 95 | 0.42 cm | 1.30 cm | 1.12 cm |
| Precisión de llegada a meta | 2.75 cm | 2.63 cm | 3.03 cm |
| Keyframes completados | 100% (50/50) | 100% (52/52) | 100% (49/49) |

### Laboratorio

Para procesar las bolsas de datos registradas en la plataforma física (`evidence/laboratorio/`):

```bash
pixi run python scripts/plot_real_comparison.py --output validation/real_comparison
```
Argumentos opcionales:

- `--output <directorio>`: Carpeta de destino (por defecto: `validation/real_comparison/`).

- `--common-window`: Normaliza el análisis a la ventana temporal mínima común entre los tres ensayos (23.8 s), permitiendo una comparativa temporal directa.

El script lee los rosbags SQLite3 (`.db3`) y genera:

1. Figura comparativa de 6 paneles (`comparacion_robot_real.png`):
   - Fila superior (3 paneles): Recorrido 2D del Teach frente al Repeat para cada modo (Encoders, LiDAR, Mixto), superpuesto sobre la nube de puntos acumulada del mapa láser.
   - Fila inferior (3 paneles):
     - *Distancia a Teach:* Desvío lateral instantáneo respecto a la trayectoria de referencia (cm).
     - *Progreso de referencias:* Porcentaje acumulado de keyframes de ruta alcanzados (0 a 100%).
     - *Alineación ICP:* RMSE residual de la correspondencia escaneo-mapa (cm).
2. Resumen en formato CSV (`metricas.csv`):
   Detalla variables cuantitativas por modo: `observed_cross_track_rms_m`, `observed_cross_track_max_m`, `reference_index_percent`, `completion_time_s` y estado de completitud.
3. Archivos de origen (`resultados.json`):
   Guarda las métricas y los hashes SHA-256 de los rosbags y las rutas utilizadas.

Los registros de Encoders (23.8 s) y LiDAR (43.7 s) son parciales: alcanzan el 18.75% y el 28.70% de las referencias, respectivamente. Mixto registra la finalización en 111.3 s, con índice final 112 y un desvío geométrico RMS estimado de 1.78 cm. La polilínea Teach suma 19.77 m estimados. Las rutas difieren y no hay una medición externa de posición, por lo que estos valores no permiten comparar precisión física ni tiempos de llegada entre los tres modos.

### Graficar una sesión

Para generar gráficos específicos de un recorrido particular inspeccionando los escaneos LiDAR y el setpoint de control en un instante dado:

```bash
pixi run python scripts/plot_run.py \
  --bag evidence/simulacion/repeat_bag_mixed \
  --route routes/sim_route_mixed.json \
  --out evidence/sim_repeat_mixed.png \
  --title "Sesion Repeat Mixto - Simulacion"
```

Parámetros de `scripts/plot_run.py`:

- `--bag <ruta>`: Ruta al directorio del rosbag (admite formato MCAP y SQLite3).

- `--route <ruta>`: *(Requerido)* Ruta al archivo JSON con la definición de la ruta (`routes/*.json`).

- `--out <ruta>`: Ruta del archivo PNG de salida (por defecto: `evidence/run_plot.png`).

- `--title <texto>`: Título personalizado de la visualización (por defecto: nombre de la carpeta del bag).

- `--time <segundos>`: Instante temporal específico a inspeccionar. Representa en la figura la pose instantánea del robot, el escaneo láser actual (rojo), los puntos de referencia almacenados en el keyframe objetivo (verde) y el setpoint comandado por el controlador.

### Videos

En la carpeta [`Videos/`](Videos/) se encuentran grabaciones de las demostraciones prácticas aceleradas a 2x:

- [`Videos/Encoders-Simulacion.mp4`](Videos/Encoders-Simulacion.mp4): Repetición en simulación por encoders puros.

- [`Videos/Lidar-Simulacion.mp4`](Videos/Lidar-Simulacion.mp4): Repetición en simulación por LiDAR 2D ICP.

- [`Videos/Mixto-Simulacion.mp4`](Videos/Mixto-Simulacion.mp4): Repetición en simulación por modo Mixto.

- [`Videos/Repeat-Mixto-Laboratorio.mp4`](Videos/Repeat-Mixto-Laboratorio.mp4): Demostración en vivo en robot real TurtleBot 4 en el laboratorio.

## Estructura del repositorio

```text
Advanced-Robotics-tp1/
├── ros_ws/src/Grupo4/tp1_teach_repeat/   # Paquete principal ROS 2
│   ├── config/params.yaml                # Parámetros calibrados para simulación (TB3)
│   ├── launch/                           # teach.launch.py, repeat.launch.py, sim.launch.py
│   ├── tp1_teach_repeat/                 # Nodos de localización, control y mapeo
│   └── test/                             # Suite de pruebas unitarias pytest (26 tests)
├── sim/                                  # Scripts de automatización para Gazebo
│   ├── common.sh                         # Rutas, tópicos y reseteo de pose
│   ├── teach.sh                          # Lanzamiento de Teach en simulación
│   └── repeat.sh                         # Lanzamiento de Repeat en simulación
├── real/                                 # Scripts para despliegue en TurtleBot 4 real
│   ├── common.sh                         # Configuración de IPs, puertos y Discovery Server
│   ├── env.sh                            # Exportación de variables DDS a terminal interactiva
│   ├── teach.sh                          # Teach físico con rosbag
│   ├── repeat.sh                         # Repeat autónomo físico con rosbag
│   ├── teleop.sh                         # Teleoperación de robot real (i/j/k/l/,)
│   └── rviz.sh                           # RViz2 conectado al robot real
├── routes/                               # Rutas guardadas en JSON (simulación y real)
├── evidence/
│   ├── simulacion/                       # Rosbags (.mcap), métricas y gráficos de simulación
│   └── laboratorio/                      # Rosbags (.db3) y métricas de ensayos reales
├── validation/real_comparison/           # Métricas CSV y figuras consolidadas de laboratorio
├── Videos/                               # Demostraciones aceleradas para la presentación
├── scripts/                              # Scripts de análisis y generación de figuras
├── params_tb4.yaml                       # Parámetros calibrados para TurtleBot 4 real
├── pixi.toml                             # Configuración del entorno reproducible Pixi
├── pixi.lock                             # Bloqueo determinista de dependencias
├── pixi-activate.sh                      # Variables de entorno y bajo nivel
└── TP1.pdf                               # Consigna oficial de la cátedra
```
