# D.8.9 — Imágenes coherentes con escenario y ejecución real

## Instalación
Extraer este ZIP en la carpeta `~/swarmbots` del anfitrión, respetando `swarm_ws/scripts/`.
Los tres archivos Python se ejecutan directamente: no requieren `colcon build`.

## Imagen offline (posiciones iniciales corregidas)

```bash
cd ~/swarm_ws
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 scripts/experimento_radio_hybrid_v2.py --scenario scripts/resultados/d8/seed_100_run_001/scenario.json --radius 0.55 --margin 0.05 --iterations 2 --output scripts/resultados/mision_100_offline_corregida.png
```

Ahora `plot_hybrid_v2_mission.plan_four` inicia desde `scenario.robots`, no de un spawn aleatorio antiguo. La opción `--seed 100` también funcionará, pero `--scenario` garantiza usar el JSON de una ejecución concreta.

## Imagen de las 4 etapas calculadas por D.8

```bash
python3 scripts/plot_mission_d8.py --run scripts/resultados/d8/seed_100_run_001
```

PNG por defecto: `.../seed_100_run_001/mission_hybrid_v2.png`.
El script solo lee los informes `protocol/<stage>_protocol_report.json` y los controles `protocol/<stage>_protocol_controls.npz`, por lo que NO recalcula ni sustituye CBS. Exige certificado de cada etapa. Los tramos de movimiento RGB-D y retorno local no están representados, ni tampoco la trayectoria realmente medida por odometría. En ejecuciones incompletas devuelve un error explicativo.

No alterar radio, umbral de proximidad ni controles operativos.
