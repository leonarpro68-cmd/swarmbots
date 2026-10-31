# D9.9.6 — preflight de Gazebo (solo lectura)

Este paquete **NO mueve robots, NO publica `/cmd_vel`, NO autoriza navegación**. Consta de verificación estática offline y observación de una sesión en vivo. La fase en vivo requiere ROS 2 Jazzy, Gazebo Harmonic y los 3 Summit quietos.

## Instalación

Copia el contenido de este ZIP a la raíz de `~/swarmbots`, conservando la carpeta `swarm_ws/`. No sobrescribe ejecutores D8/D9. Los archivos en `scripts/d996_reference` son una referencia congelada de `seed_100_run_007`; no son datos nuevos obtenidos en Gazebo.

## Verificación estática (sin ROS 2)

Desde `~/swarmbots/swarm_ws`:

```bash
python3 scripts/d996_preflight.py \
  --reference-report scripts/auditoria_identidad_d99.json \
  --world-sdf scripts/d996_reference/world.sdf \
  --scenario scripts/d996_reference/scenario.json \
  --trajectory scripts/d996_reference/pickup1_d994_NON_EXECUTABLE.npz \
  --conversion-report scripts/d996_reference/pickup1_d994_conversion_report.json \
  --offline-preflight scripts/d996_reference/pickup1_d995_preflight.json \
  --output scripts/resultados/d996_static.json
```

Estado esperado: `PASS_STATIC_LIVE_NOT_RUN`. Esto NO significa que Gazebo coincida.

## Sesión en vivo

1. Abre Gazebo con el escenario `seed_100_run_007` (el script espera el mundo `world_demo` y los modelos `summit0`, `summit1`, `summit2`). Utiliza tu configuración probada de arranque, sin iniciar el ejecutor de misión. Ejemplo condicionado a que el launch disponga de los parámetros indicados:

```bash
cd ~/swarmbots/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch swarm_worlds swarm_cbs_grasp_base.launch.py \
  seed:=100 \
  scenario:="$PWD/scripts/d996_reference/scenario.json" \
  world_sdf:="$PWD/scripts/d996_reference/world.sdf" \
  hybrid_enabled:=false headless:=false
```

2. En una segunda terminal, tras comprobar que los tres Summit están detenidos:

```bash
cd ~/swarmbots/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/d996_preflight.py \
  --reference-report scripts/auditoria_identidad_d99.json \
  --world-sdf scripts/d996_reference/world.sdf \
  --scenario scripts/d996_reference/scenario.json \
  --trajectory scripts/d996_reference/pickup1_d994_NON_EXECUTABLE.npz \
  --conversion-report scripts/d996_reference/pickup1_d994_conversion_report.json \
  --offline-preflight scripts/d996_reference/pickup1_d995_preflight.json \
  --live --observe-seconds 8 \
  --output scripts/resultados/d996_live.json
```

Esperado SOLO cuando todas las verificaciones pasan: `PASS_STATIC_AND_LIVE_INERT`. Si rechaza, revisa `problems` en el JSON. En ambos casos `execution_authorized=false`.

## Qué verifica y qué NO

- Coincidencia SHA-256 de `world.sdf`, `scenario.json` y trayectoria convertida respecto a la referencia certificada.
- Presencia y estado satisfactorio de los reportes de conversión y preflight offline.
- En vivo: listado de Summit de Gazebo, `/clock` creciente, odometría fresca, velocidad de reposo, posición de inicio D9.9 con tolerancia 5 cm, y nombres declarados de frames.
- **NO certifica** que Gazebo haya cargado byte a byte el SDF indicado, ni las transformaciones TF, ni la correspondencia geométrica real del mapa. Tampoco revisa dinámicamente la conexión completa de TF; requiere auditoría adicional si se necesita un certificado estricto de TF.
- Si el arranque real supera 5 cm de distancia frente al origen medido de `run_007`, el reporte rechaza: NO mover robots para ajustarlo a la referencia sin un procedimiento seguro; se recalculan trayectorias offline desde la odometría nueva.

## Tests de control (sin ROS)

```bash
python3 -m unittest discover -s scripts/tests -p test_d996_preflight.py -v
```

No habilitar ninguna publicación de velocidades con este módulo. La futura navegación requiere una evaluación independiente de seguridad.
