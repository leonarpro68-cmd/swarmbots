# D.9.6C.2B — Planificación en vivo y cadena D9 read-only

**NO mueve robots.** No ejecuta la misión, no crea publicadores de velocidad ni modifica D8.

El programa `swarm_ws/scripts/d96c2b_live_pipeline.py` invoca, de forma secuencial:
1. `d6_mission_state.py` (inicia un estado nuevo SOLO para `pickup1`, o copia un ledger D6 proporcionado por el operador para etapas posteriores).
2. `prepare_hybrid_v2_d6_stage.py` (captura la odometría actual y calcula CBS + Hybrid V2 certificados; *sin movimiento*).
3. `retime_hybrid_v2_d51.py` y `convert_d51_protocol_pieces.py` para reconstruir el protocolo de referencia D8.
4. `d92_smooth_retime.py` sobre esa NUEVA referencia.
5. `d94_convert_offline.py` (con 4 subdivisiones y muestreo 0.005 s).
6. `d95_protocol_preflight.py` para comprobar las curvas CONVERTIDAS.

Cada etapa debe superar el certificado específico y generar su propio informe. Si falla, registra `REJECTED`, sale con código 2 y NO habilita ningún movimiento. Nunca usar sus salidas como comandos.

## Instalar

Fuera de Docker: `unzip -o ~/Descargas/swarm_D9_6C_2B_planificador_vivo_readonly.zip -d ~/swarmbots`

Dentro: `cd ~/swarm_ws && python3 scripts/tests/test_d96c2b_pipeline.py`

## Iniciar Gazebo

Usa la simulación de almacén para la semilla 100 que ya tienes abierta, **sin la misión**, con `/clock`, `/summit0/odom`, `/summit1/odom`, `/summit2/odom` visibles. Las posiciones del escenario deben coincidir con el JSON utilizado realmente al lanzarla. Los Summit deben estar detenidos. Es necesario que el operador haya verificado el significado global de `summitN/odom` antes de optar por el modo experimental namespaced (esto **NO** crea ni sustituye TF).

## Ejecutar solo `pickup1`

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"

python3 scripts/d96c2b_live_pipeline.py \
  --scenario scripts/resultados/d8/seed_100_run_007/scenario.json \
  --stage pickup1 \
  --accept-namespaced-world-odom \
  --output scripts/resultados/d9_live/seed_100_pickup1_001
```

El directorio `--output` **NO debe existir** previamente. Cambia el sufijo `001` para cada intento. Si las coordenadas `summitN/odom` no se han verificado en relación con el mundo, NO uses `--accept-namespaced-world-odom` y no ejecutes este planificador; verifica primero los marcos.

## Resultados

- `d96c2b_report.json`: `PASS_READ_ONLY_LIVE_REPLAN` o `REJECTED`, **siempre** `execution_authorized=false`.
- `pipeline.log`: salida completa del subprocess que falló, si hay error.
- `live/pickup1_live_report.json`, `live/pickup1_live_controls.npz`: resultados frescos D8.
- `mission/pickup1/protocol/*`: protocolo D8 recertificado para el adaptador offline (NO se publica a ROS).
- `d92_offline_report.json`: nuevo escalado suave sobre controles frescos.
- `d94_conversion/pickup1_converted_NON_EXECUTABLE.npz`: segmentos independientes.
- `d95_preflight/d95_summary.json`: comprobaciones offline del plan convertido.

**Restricciones:** sólo se lanza `pickup1` sin ledger adicional. Para `delivery1`, `pickup2` y `delivery2` es obligatorio aportar `--state` real y validado que refleje entregas físicas; no se falsifican estados. La comprobación integral de transformaciones TF, tiempo transcurrido desde captura hasta ejecución, errores de seguimiento y parada coordinada continúa pendiente. No se garantiza una mejora del 33,8 % en una nueva planificación.
