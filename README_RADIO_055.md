# Experimento radio 0.55 m — Hybrid V2 / semilla 123

Script **offline y experimental**: no modifica la geometría base persistida, el CBS, los launch de ROS ni Gazebo. Sobrescribe en memoria `geometry_reference.CLEARANCE_WALL_M` y `geometry_corridors.R` de manera temporal durante la planificación (restaura después), y certifica con los mismos valores. Se mantiene reserva adicional 0.05 m; separación requerida = 2*0.55+0.05 = 1.15 m. No confundir este ensayo con validación de las dimensiones físicas de un Summit.

Prerequisitos: D.1 + parche de una pieza + D.2, y sus dependencias Python.

Instalar ZIP desde Ubuntu fuera de Docker: `unzip -o ~/Descargas/experimento_hybrid_v2_radio055.zip -d ~/swarmbots`

Dentro de Docker:
```
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 scripts/experimento_radio_hybrid_v2.py \
  --seed 123 --radius 0.55 --margin 0.05 --iterations 2 \
  --output scripts/resultados/mision_123_hybrid_v2_r055.png \
  --json-output scripts/resultados/mision_123_hybrid_v2_r055.json
```

Devuelve código 2 si alguna etapa es REJECTED (aunque exporta PNG/JSON). La prueba parcial observada certificó pickup1, delivery1 y pickup2; el entorno de evaluación llegó al tiempo máximo durante delivery2 y no hay certificación de misión completa todavía. Cada ejecución puede consumir bastante tiempo.

**Advertencia geométrica:** Los obstáculos del CBS siguen usando sus exclusiones previas y no se reinterpretan con 0.55 m; si se desea consistencia física completa hay que separar radio de huella y radio de margen en ambos planificadores y corroborar colisiones SDF. La certificación actual es solo dentro del modelo geométrico experimental.
