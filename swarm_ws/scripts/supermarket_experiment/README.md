# SwarmBots · Fase 6: evaluación experimental (v1)

**Herramientas de observación; no controlan robots, ni publican `cmd_vel`.** Compatible con ROS 2 Jazzy y la simulación `supermarket` de semilla 100. No modifica la misión fija ni MILP.

## Instalación en Ubuntu, fuera de Docker

Descomprime el ZIP en `~/Descargas` y ejecuta:

```bash
bash ~/Descargas/supermarket_fase6_evaluacion_v1/install_on_host.sh ~/swarmbots/swarm_ws
```

## Protocolo propuesto

La comparación precisa **repetir** misión fija y misión MILP con el recorder encendido desde antes del primer movimiento hasta después de la última descarga. La ejecución exitosa anterior no implica que se hayan guardado automáticamente todas las mediciones. No mezclar archivos de diferentes sesiones de Gazebo.

**Dentro de Docker**, después de lanzar Gazebo y `supermarket_support.launch.py`, y antes de mover cualquier robot:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
RUN="$HOME/swarm_ws/scripts/resultados/experiments/fixed_seed100_trial01"
# Para la segunda prueba, sustituir fixed por milp y reiniciar Gazebo
python3 scripts/supermarket_experiment/record_odom.py --output "$RUN/odom.csv"
```

Deja el grabador ejecutándose en esa terminal. En otra terminal marca los eventos (cada invocación consulta `/clock`):

```bash
python3 scripts/supermarket_experiment/mark_event.py --run-dir "$RUN" --event mission_start
```

Justo antes y justo después de la **navegación** de cada etapa (no del agarre físico) registra, por ejemplo:

```bash
python3 scripts/supermarket_experiment/mark_event.py --run-dir "$RUN" --event pickup1_nav_start
# Ejecutar el coordinador ya validado de pickup1, desde otra terminal
python3 scripts/supermarket_experiment/mark_event.py --run-dir "$RUN" --event pickup1_nav_end
```

Cuando las **tres maniobras físicas** de esa etapa estén terminadas y verificadas visualmente, marca `pickup1_physical_done`. Continúa con `delivery1`, `pickup2`, `delivery2`. Para medir exclusivamente la parte física, también puedes registrar `pickup1_physical_start` al iniciar la primera maniobra, y análogamente las otras. Al finalizar y comprobar seis entregas:

```bash
python3 scripts/supermarket_experiment/mark_event.py --run-dir "$RUN" --event mission_end
```

Detén el grabador con `Ctrl+C`. Luego ejecuta (no requiere Gazebo):

```bash
python3 scripts/supermarket_experiment/analyze_run.py --run-dir "$RUN"
```

Repite para `milp_seed100_trial01` **desde un Gazebo recién iniciado** con mismos límites 2 m/s y 2 m/s², geometría 9 cm, parámetros D8 y condiciones equivalentes. Para comparar:

```bash
python3 scripts/supermarket_experiment/compare_runs.py \
  --fixed ~/swarm_ws/scripts/resultados/experiments/fixed_seed100_trial01/summary.json \
  --milp ~/swarm_ws/scripts/resultados/experiments/milp_seed100_trial01/summary.json \
  --output-dir ~/swarm_ws/scripts/resultados/experiments/comparison_01
```

Se producen `comparison.csv` y `comparison.json`. La diferencia positiva MILP−fija indica aumento; negativa, disminución.

## Interpretación y límites

- `mission_duration_sim_s` es tiempo transcurrido en el reloj de **simulación** entre marcas del operador. No se llama tiempo real de pared. Incluye pausas humanas, comprobaciones y latencias entre etapas: conviene distinguirlo de tiempos de ejecución puramente automáticos.
- `navigation_observed_s` corresponde a las marcas manuales de inicio/fin; incluye el tiempo de armado, demoras de introducción de comandos y observación. **No sustituye los tiempos de inicio/fin registrados por el coordinador.**
- `distance_m_observed` es longitud de trayectoria de odometría muestreada, no distancia real perfecta; solo es comparable si se registraron sesiones completas con frecuencias equivalentes. Los `gaps_over_1s` advierten huecos. Puede incluir reposicionamiento local D8.
- `peak_speed_m_s_observed` es el máximo de velocidades publicadas por odometría, no medida independiente de contacto.
- `physical_confirmed` registra lo que el operador marcó después de verificar las tres maniobras, no una detección automatizada.
- Este paquete **no calcula aún** error Bézier por etapa ni distancia mínima a estanterías o robots. Para ello existen `supermarket_telemetria_pickup1_v1/analyze.py` y reportes de los controladores; debe extenderse para las cuatro etapas con NPZ MILP y fijos distintos.
- Para afirmar una mejora con evidencia experimental se requieren repeticiones, mismos parámetros, registro de errores de agarre y dispersión; un único ensayo no demuestra robustez.

Pruebas offline:

```bash
python3 -m unittest discover -s ~/swarm_ws/scripts/supermarket_experiment/tests -v
```

Las pruebas se incluyen en el ZIP, aunque el instalador deliberadamente instala solo los programas y README; para probar usa `tests` extraído localmente.
