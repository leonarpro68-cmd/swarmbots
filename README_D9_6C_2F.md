# D.9.6C.2F — Comparador estático Gazebo ↔ ROS 2 (solo lectura)

Instalar en host desde `~/swarmbots`:

```bash
unzip -o ~/Descargas/swarm_D9_6C_2F_comparador_estatico.zip -d ~/swarmbots
```

Dentro de Docker (`~/swarm_ws`):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/tests/test_d96c2f_compare_frames.py
python3 scripts/d96c2f_compare_frames.py \
  --output scripts/resultados/d9_live/d96c2f_static_frames.json \
  --observe-seconds 5
```

Es imprescindible que Gazebo publique `/world/world_demo/pose/info`, el bridge publique `/summit{0,1,2}/odom` y `/clock`, y los robots estén inmóviles.

El código no crea publicadores ROS, no envía velocidades, no publica TF y no llama servicios de modificación.

**Interpretación:** `STATIC_FRAME_CONSISTENCY_OBSERVED_NOT_PROVEN` solo indica coincidencia estática de la pose del modelo Gazebo con `base_footprint` ROS, dentro de 5 cm y 3 grados. Las poses no están emparejadas por marca temporal y el origen del modelo Gazebo puede diferir de `base_footprint`. Si aparece `REJECT_FRAME_COMPARISON`, revisar conteo de muestras y datos por robot. En cualquier resultado `coordinate_frame_equivalence_proven=false` y `execution_authorized=false`.

**Siguiente fase:** verificar si existe una transformación fija entre mundo y odometría, idealmente mediante relaciones del plugin de Gazebo o comparación temporal de un movimiento controlado aislado en un entorno seguro. Sin eso, no habilitar D.9.6C.2 con movimiento.
