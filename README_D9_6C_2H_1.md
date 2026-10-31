# SwarmBots — D.9.6C.2H.1

## Alcance (SOLO LECTURA)

Este parche **no ejecuta D.8, no genera movimiento, no publica `/cmd_vel`, ni publica TF**.
Tampoco cambia los archivos Xacro, SDF o launch originales. Audita su contenido.

Se verifican conjuntamente:

1. El Xacro `summit_xl_omni.urdf.xacro` define **OdometryPublisher** y **MecanumDrive** con odometrías distintas.
2. `sim_summit.launch.py` usa dicho Xacro y puentea la odometría de física hacia `/<robot>/odom` y la de MecanumDrive hacia `/<robot>/encoder_odom`.
3. El **SDF generado** para cada Summit realmente contiene los plugins y las etiquetas esperadas; por defecto se examina `/tmp/swarm_summit/summitN.sdf`.
4. El informe anterior D.9.6C.2G indica consistencia dinámica observada, con suficiente desplazamiento, muestras emparejadas y errores pequeños.
5. El observador D.9.6C.2G recibe una corrección del almacenamiento del reloj: **cuenta todas** las muestras, detecta resets incluso después de la muestra 40 000, y no consume memoria lineal por el reloj.

**NUNCA interpreta estas comprobaciones como certificado formal de TF o autorización para ejecutar.**

## Instalación

Anfitrión (fuera de Docker, en la carpeta raíz de SwarmBots):

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_D9_6C_2H_1_marcos_reloj_readonly.zip -d ~/swarmbots
```

Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/tests/test_d96c2h1_config_audit.py -v
python3 scripts/tests/test_d96c2g_dynamic_frames.py -v
```

## Requisitos

- Los scripts originales en `src/summit_xl_description/robots/summit_xl_omni.urdf.xacro` y `src/swarm_worlds/launch/sim_summit.launch.py`.
- Los **tres** archivos SDF generados durante el último arranque en el mismo contenedor: `/tmp/swarm_summit/summit0.sdf`, `summit1.sdf`, `summit2.sdf`.
- El informe `scripts/resultados/d9_live/d96c2g_dynamic_frames.json` de la prueba exitosa D.9.6C.2G (no hace falta volver a mover los robots).

Comprobar la existencia de los SDF:

```bash
ls -lh /tmp/swarm_summit/summit{0,1,2}.sdf
```

Si los SDF no existen, el auditor los rechazará con claridad. No fabriques SDF ni transformaciones para hacer pasar la prueba; identifiquemos por qué desaparecieron primero. No hace falta ejecutar D.8 otra vez solo para correr el auditor.

## Ejecutar auditor

```bash
cd ~/swarm_ws
python3 scripts/d96c2h1_config_audit.py \
  --dynamic-report scripts/resultados/d9_live/d96c2g_dynamic_frames.json \
  --output scripts/resultados/d9_live/d96c2h1_configuration_report.json
```

Estado esperado solo si toda la configuración y evidencias coinciden:

`PASS_CONFIG_AND_DYNAMIC_OBSERVATION_NOT_TF_CERTIFIED`

De otro modo:

`REJECT_CONFIGURATION_OR_EVIDENCE`

En **ambos estados** el reporte tendrá:

```json
{
  "coordinate_frame_equivalence_proven": false,
  "tf_tree_certified": false,
  "execution_authorized": false,
  "publishers_created": 0,
  "commands_sent": 0
}
```

## Datos técnicos verificados en archivos del proyecto

- Xacro (mecanum): `/model/${ns}/mecanum_odom` y `/model/${ns}/mecanum_tf`.
- Xacro (OdometryPublisher de física): `/model/${ns}/odometry` y `/model/${ns}/tf`.
- Las etiquetas del último son `${ns}/odom` y `${ns}/base_footprint`.
- El launch puentea odometría de física como `/<ns>/odom` y odometría mecanum como `/<ns>/encoder_odom`.

Los archivos `sim_summit.launch.py` y `summit_xl_omni.urdf.xacro` son la fuente local, no se editan.

## Limitaciones que se conservan

1. Coincidencia `world ↔ odom` observada bajo una configuración D.8 no implica TF `map ↔ summitN/odom` publicado/certificado.
2. La correspondencia observada de poses de modelo frente a `base_footprint` puede incluir offsets 3D; el seguimiento futuro es planar y tendrá que respetarlo.
3. La comparación D.9.6C.2G anterior conservó solo 40 000 marcas de reloj. Este parche corrige el código para nuevas capturas, pero **no recupera marcas descartadas de una prueba anterior**.
4. El auditor lee archivos generados: **no prueba que los procesos Gazebo en ejecución hayan cargado esos mismos bytes**. Tampoco representa un permiso de movimiento.
5. Para D.9.6C.2H.2 aún se requiere la exclusión de controladores simultáneos, un preflight de origen reciente, supervisión con actualización continua y parada coordinada.

## Pruebas

15 pruebas del comparador D.9.6C.2G (12 originales + 3 nuevas del reloj) y 10 pruebas del auditor D.9.6C.2H.1. Todas aprobaron en el entorno de preparación. La prueba real sobre los archivos SDF generados del usuario queda por hacer.
