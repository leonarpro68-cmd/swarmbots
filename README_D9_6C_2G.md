# SwarmBots — D.9.6C.2G | Comparación dinámica Gazebo ↔ ROS 2

**Propósito:** recoger las poses globales de `summit0`, `summit1` y `summit2` de
`/world/world_demo/pose/info` y sus odometrías `/summitN/odom`; emparejar **por
marca de tiempo de simulación**, interpolar la odometría y comparar X, Y y yaw.

Esta herramienta es **pasiva**: crea suscripciones ROS, lee `gz topic -e`,
**no** crea publicadores, **no** llama servicios de modificación,
**no** emite `/cmd_vel` y **no** publica TF. Para observar movimiento real en
Gazebo se utiliza **el ejecutor D.8 previamente probado**, no D.9.2.

## 1. Instalar (terminal HOST, fuera de Docker)

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_D9_6C_2G_comparacion_dinamica.zip -d ~/swarmbots
```

El ZIP añade únicamente:

- `swarm_ws/scripts/d96c2g_dynamic_frames.py`
- `swarm_ws/scripts/tests/test_d96c2g_dynamic_frames.py`

No modifica los ejecutores, controladores, launch files ni D.8/D.9.

## 2. Comprobar código (dentro de Docker)

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m py_compile scripts/d96c2g_dynamic_frames.py
python3 scripts/tests/test_d96c2g_dynamic_frames.py -v
```

**Los tests simulan mensajes, no constituyen una certificación real de Gazebo.**

## 3. Prueba dinámica: dos terminales dentro de Docker

Antes de iniciar, **detén con Ctrl+C** el launch manual de Gazebo y los
puentes que corresponden a esa simulación. No superpongas dos mundos.

**Terminal 1 — misión D.8 estable y completa (LOS ROBOTS SE MOVERÁN).**

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 scripts/run_swarm_seed_d8.py --seed 100 --auto-confirm --execute
```

El argumento `--execute` ejecuta la misión y mueve físicamente los modelos
en el simulador. Comprueba el escenario antes de usarlo; no lo ejecutes en
hardware. No hay que ejecutar D.9.2 ni controlar robots desde una tercera
terminal.

**Terminal 2 — observador pasivo.** Cuando la Terminal 1 ya haya arrancado
Gazebo, comprueba:

```bash
gz topic -l | grep '/world/world_demo/pose/info'
ros2 topic list | grep -E '^/(clock|summit[012]/odom)$'
```

Si aún no aparecen, espera a que termine de iniciarse Gazebo. Luego ejecuta
el observador **mientras D.8 está moviendo los robots**:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/d96c2g_dynamic_frames.py \
  --output scripts/resultados/d9_live/d96c2g_dynamic_frames.json \
  --observe-seconds 240
```

Puedes interrumpir la misión D.8 ante un comportamiento inesperado; el
observador **no controla** el movimiento ni puede detenerla.

## 4. Informe y criterios

Archivos resultantes:

- `scripts/resultados/d9_live/d96c2g_dynamic_frames.json`
- `scripts/resultados/d9_live/d96c2g_dynamic_frames_matched.csv`

Se exige para **cada** Summit:

- Al menos 10 muestras Gazebo–odometría emparejadas por timestamp de simulación.
- Puntos de odometría alrededor del timestamp Gazebo a no más de 40 ms por lado.
- Al menos 0,25 m de recorrido y 0,125 m de extensión espacial en las muestras emparejadas.
- Coincidencia dinámica: error máximo XY ≤ 7,5 cm, error máximo yaw ≤ 4°.
- Marcos de odometría etiquetados como `summitN/odom` y `summitN/base_footprint`.
- Reloj `/clock` avanzando sin retrocesos.

**Resultado satisfactorio (solo evidencia observacional):**

`DYNAMIC_WORLD_ODOM_CONSISTENT_OBSERVED_NOT_TF_PROVEN`

**Rechazos útiles:** `REJECT_GAZEBO_POSE_TIMESTAMPS_MISSING`,
`REJECT_INSUFFICIENT_DYNAMIC_EVIDENCE`, `REJECT_CLOCK_NOT_PROGRESSING_OR_RESET`,
`REJECT_DYNAMIC_FRAME_COMPARISON`, `REJECT_CAPTURE_ERROR`.

Si Gazebo no incluye timestamps en sus poses, el programa rechaza
explícitamente: no intenta emparejar usando la hora de llegada.
Para diagnóstico ejecuta:

```bash
timeout 5s gz topic -e -t /world/world_demo/pose/info | head -n 90
```

En ocasiones es necesario consultar un tópico alternativo:
`--pose-topic /world/world_demo/dynamic_pose/info`; debe contener la pose
de los **modelos** `summitN` y timestamps propios para aprobar.

**No confundir estado positivo con permiso de movimiento.** Aun si el JSON
informa consistencia dinámica, mantiene siempre:

```json
{
  "coordinate_frame_equivalence_proven": false,
  "execution_authorized": false,
  "ros_executable": false,
  "publishers_created": 0,
  "commands_sent": 0
}
```

La pose mundial del *modelo* puede diferir de la pose de
`base_footprint`, y la coincidencia medida no construye ni publica una
cadena TF formal. La etapa posterior inspeccionará la configuración SDF y
la definición de los frames antes de habilitar cualquier controlador D.9.2.
