# D.9.8C.4 — CBS discreto por oleadas (offline, solo lectura)

Este parche **no mueve robots ni acciona pinzas**. Reutiliza `swarm_behavior.swarm_planning.cbs` y la geometría original de la bodega. Para cada oleada calcula CBS para los robots activos y retira del grafo nodos y aristas que pasan demasiado cerca de los Summit en espera. Usa un margen conservador de 1,55 m entre centros en la previsualización. Las acciones son *ticks abstractos*; no representan tiempos reales.

## Instalación

En el anfitrión: `unzip -o ~/Descargas/swarm_D98C4_CBS_oleadas_offline.zip -d ~/swarmbots`

Dentro del Docker, sin Gazebo:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/scripts:$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m unittest discover -s scripts/tests -p test_d98c4_wave_cbs_preview.py -v

python3 scripts/d98c4_wave_cbs_preview.py \
 --snapshot scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_live_snapshot.json \
 --waves scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_wave_proposal.json \
 --geometry scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_wave_geometry.json
```

Resultado: `scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_wave_cbs_preview.json`.

## Resultado probado con los JSON enviados

- Oleada 1: CBS discreto encontró rutas (Summit 2 a azul, Summit 0 a verde, Summit 1 estacionado).
- Oleada 2: `summit1:SNAP_TOO_FAR`: el punto nominal de llegada al depósito verde queda inaccesible en la cuadrícula cuando se mantiene estacionado Summit 0 en el final estimado de la oleada 1. **Esto requiere una etapa de aparcamiento/despeje de Summit 0 después de liberar su basura**, o un objetivo de descarga distinto. No debe reducirse el margen de separación como atajo.
- La segunda oleada se basa en posiciones finales *estimadas* de la primera, no en una lectura viva de odometría; después de ejecutar la oleada 1 hará falta planificar de nuevo.

## Restricciones

El cierre anterior de Gazebo destruyó el estado de cargas unidas. Las rutas discretas no son aptas para el seguidor D.9.7C.1, ni están certificadas por Hybrid V2 / D.9.2. No se actualiza el estado D6 ni se autoriza ejecución, liberación o recogida. El CSV/JSON de rutas es una previsualización y no reemplaza la comprobación de articulaciones físicas, odometría fresca y pausas/oleadas reales.
