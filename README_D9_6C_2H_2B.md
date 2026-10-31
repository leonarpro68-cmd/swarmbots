# SwarmBots — D.9.6C.2H.2B: supervisor de seguridad continuo (SOLO SOMBRA)

## Alcance

Esta entrega incorpora **`swarm_ws/scripts/d96c2h2b_guarded_shadow.py`** y
**`swarm_ws/scripts/tests/test_d96c2h2b_guarded_shadow.py`**. No reemplaza D.8,
los lanzadores, Xacro, SDF, D.9.2 ni los certificados D.9.5. No crea publicadores
ROS, no envía `/cmd_vel`, no transmite TF ni detiene físicamente robots. Para
esta versión, los Summit **deben permanecer inmóviles** en el inicio de `pickup1`.

## Estados

`DISARMED → ARMED → RUNNING_SHADOW → STOPPED` (aprobación de observación).
Cualquier condición inválida activa `FAULT` **enclavado**, que no se rearma
manualmente en el mismo proceso. `ARMED` y `RUNNING_SHADOW` son **estados
internos sin autoridad de movimiento**, no equivalen al armado de motores.

Antes de entrar en estado `ARMED` se reutilizan D.9.6C.2H.1, el control de
integridad y el manifiesto D.9.5 mediante D.9.6C.2H.2A / D.9.6C.2C. Durante
la observación se reevalúan en tiempo real:

- reloj recibido y **reloj progresando**, reinicio del reloj; watchdog wall 0,75 s;
- odometría fresca y marcas temporales consistentes con `/clock`, watchdog wall 0,4 s;
- nombres de `frame_id`/`child_frame_id` y cuaterniones válidos;
- velocidad **<=0,03 m/s**, distancia al inicio **<=0,05 m**, deriva yaw **<=3°**;
- separación XY **>=1,15 m** para los 3 pares;
- consulta periódica a los **11 tópicos** potenciales de velocidades cada ~0,25 s; rechazo si aparece algún publicador;
- vigencia de la captura original D.9.6C.2B (<=120 s de tiempo de simulación) en el prearmado.

**Advertencia importante:** un muestreo periódico de grafo ROS no constituye
exclusividad efectiva. `FAULT` no garantiza parada física si otro proceso controla
un Summit. La infraestructura de parada activa / watchdog Gazebo todavía no
existe en esta entrega. No usar para movimiento. La coincidencia numérica
Gazebo–odom fue observada, pero no certifica TF.

## Instalación

**Fuera de Docker**, en el equipo anfitrión:

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_D9_6C_2H_2B_supervisor_sombra.zip -d ~/swarmbots
```

**Dentro de Docker**:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/tests/test_d96c2h2b_guarded_shadow.py -v
```

## Preparación de Gazebo

Mantener **un solo** Gazebo corriendo con `seed 100`, sus 3 Summit quietos,
`/clock` y `/summit{0,1,2}/odom` activos. No ejecutar la misión D.8.
Cualquier nodo D.8 que publique `/cmd_vel` deberá estar detenido antes del test.
No es suficiente que no haya movimientos; debe haber cero publicadores de los
tópicos de velocidad vigilados.

Verificar ejemplos de nodos de velocidad:

```bash
ros2 topic info /summit0/cmd_vel -v
ros2 topic info /summit1/cmd_vel -v
ros2 topic info /summit2/cmd_vel -v
```

## Plan fresco (NO reutilizar plan caducado)

Si desde el plan `_003` ya pasaron más de 120 segundos de simulación, generar
una nueva carpeta de planificación. El comando **no ejecuta movimiento**:

```bash
cd ~/swarm_ws
python3 scripts/d96c2b_live_pipeline.py \
  --scenario scripts/resultados/d8/seed_100_run_007/scenario.json \
  --stage pickup1 \
  --accept-namespaced-world-odom \
  --output scripts/resultados/d9_live/seed_100_pickup1_004
```

Si `_004` ya existe, usar `_005` (o nombre nuevo); nunca sobrescribir un plan.
El pipeline debe devolver `PASS_READ_ONLY_LIVE_REPLAN`.

## Ejecución pasiva (justo después de planificar)

```bash
python3 scripts/d96c2h2b_guarded_shadow.py \
  --run scripts/resultados/d9_live/seed_100_pickup1_004 \
  --stage pickup1 \
  --config-report scripts/resultados/d9_live/d96c2h1_configuration_report.json \
  --warmup-seconds 2 \
  --observe-seconds 5
```

El resultado satisfactorio es `PASS_CONTINUOUS_HOLD_SHADOW_ONLY`, con
`state=STOPPED`, cero comandos y la lista de transiciones
`DISARMED → ARMED → RUNNING_SHADOW → STOPPED`.

Se crean únicamente archivos locales:

```text
scripts/resultados/d9_live/seed_100_pickup1_004/d96c2h2b_guard/
  pickup1_guard_report.json
  pickup1_hold_observations.csv
```

Si aparece `FAULT_OR_REJECTED_SHADOW_ONLY` o `REJECTED`, consultar `faults`,
`initial_check_reasons` o `error` en JSON. Si indica `CLOCK_*`, revisar `/clock`;
si `OTHER_VELOCITY_PUBLISHER`, cerrar el controlador ajeno;
si `LEFT_START`, generar un nuevo plan desde la posición actual.

## Criterio de avance

Una aprobación demuestra que nuestro **supervisor pasivo** detecta condiciones
de reposo y ejecuta transiciones supervisadas. **No valida seguimiento D.9.2,
limites de aceleración bajo movimiento, exclusividad efectiva ni un mecanismo
real de parada.** Esas propiedades requieren la siguiente fase y pruebas de
inyección de fallos en Gazebo con parada garantizada antes de mover robots.
