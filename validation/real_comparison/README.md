# Comparación descriptiva del robot real

Reproducir desde la raíz del repositorio:

```bash
pixi run --frozen python scripts/plot_real_comparison.py
```

Para acotar los tres modos a la menor ventana Repeat disponible:

```bash
pixi run --frozen python scripts/plot_real_comparison.py --common-window --output validation/real_comparison_common
```

Esta variante muestra los primeros 23,844017205 s desde el primer status de cada
Repeat. Recorta todos los datos a ese intervalo y ajusta los tres mapas con la
misma escala. Muestra Teach solo hasta el objetivo activo al final del intervalo.
Es un corte por tiempo, no por una distancia o referencia idéntica. No interpola
ni extrapola muestras hasta el borde. `completed_in_recording` describe el registro
original y `completed_in_window` la ventana seleccionada. `completion_time_s`
conserva el tiempo de finalización original cuando existe, aunque quede fuera
del intervalo mostrado.

## Fuentes y asociación

Se leen exclusivamente los SQLite locales `evidence/laboratorio/repeat_bag_{encoders,lidar,mixed}`
y las rutas `routes/lab_route_{encoders,lidar,mixed}.json`. No se modifican los bags.
`resultados.json` incluye sus SHA-256. No se usa el script preexistente
`plot_comparison_marauder.py` ni sus archivos externos.

El modo declarado en la ruta debe coincidir con `/tnr/status`.
Además se reconstruye el objetivo global con la pose `/tnr/odom` y el
`/tnr/setpoint` del mismo sello de medición. Su distancia al keyframe guardado
más cercano debe tener P95 menor a 3 cm. Esta prueba confirma la correspondencia
geométrica de los objetivos publicados con cada ruta.

## Lectura del gráfico

- Arriba: Teach guardado, referencias, mapa de escaneos transformados por las
  poses Teach y trayectoria estimada Repeat. Se usan límites y escala iguales.
  Cada panel tiene su propio marco local. No se realinea Repeat a posteriori.
- Abajo a la izquierda: distancia mínima de cada pose Repeat a los segmentos
  de la polilínea Teach. Es una distancia geométrica, no una correspondencia
  temporal ni una medición de precisión física. En cruces puede asociar otra
  parte de la ruta. No mide orientación ni orden de recorrido.
- Abajo al centro: índice de referencia dividido por el índice final. Es avance
  por referencias, no porcentaje de longitud ni de tiempo de la ruta.
- Abajo a la derecha: RMSE tal como se publica en `/tnr/alignment`. Mide
  consistencia entre nubes. No es error de posición ni demuestra que el láser
  esté corrigiendo la pose en modo encoders.

Todos los tiempos usan recepción en el bag, relativa al primer `/tnr/status`
del Repeat. En Mixto el análisis corta en la primera finalización y conserva
el status publicado inmediatamente después del Bool, para incluir el índice
final. El tiempo de llegada en las métricas es el de ese primer Bool verdadero.
En los otros modos termina en el último status disponible.

## Límites de la comparación

Encoders y LiDAR no publican `done=True` en estos registros. Deben tratarse como
observaciones parciales, sin atribuir una causa al fin de la grabación. Mixto sí
registra finalización. Las ventanas observadas no son tiempos de llegada
comparables. No se extrapolan los segmentos faltantes.

Las rutas Teach difieren y solo hay una grabación Repeat seleccionada por modo.
No hay réplicas controladas ni ground truth independiente. `/tnr/odom` usa el
estimador que se evalúa, por lo que un desvío pequeño no demuestra ausencia de
deriva física. No se mezcla con `/tb4_1/odom` o `/tb4_2/odom`: hacerlo alternaría
estimadores y orígenes distintos dentro de una misma curva.

`metricas.csv` contiene ventanas, finalización, referencias y desvíos descriptivos.
El RMS geométrico pondera por tiempo mediante integración trapezoidal para evitar
que una frecuencia de publicación mayor pese más en el promedio. Las métricas
parciales no constituyen una clasificación de los métodos. La longitud Teach
es la suma de segmentos estimados, no una longitud física medida.
