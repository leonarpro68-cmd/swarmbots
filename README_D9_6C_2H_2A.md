# SwarmBots D.9.6C.2H.2A — compuerta de seguridad, solo lectura

Este parche **NO es un ejecutor**. Es una comprobación previa para el futuro
seguidor de Bézier independientes. **CERO** publicadores y comandos de movimiento;
**no proporciona una parada de emergencia activa**. No modifica D.8 ni TF.

## Instalar (en anfitrión)

```
cd ~/swarmbots
unzip -o ~/Descargas/swarm_D9_6C_2H_2A_compuerta_solo_lectura.zip -d ~/swarmbots
```

## Pruebas locales (Docker)

```
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/tests/test_d96c2h2a_safety_gate.py -v
```

## Condiciones de la prueba

1. Gazebo con mundo `world_demo`, semilla 100, tres robots **quietos** y los puentes
   `/clock` y `/summitN/odom` funcionando. No ejecutar la misión D.8 activa
   a la vez. Al reabrir Gazebo, asegurarse de cargar el escenario y mundo de
   la semilla 100, sin reutilizar un mundo ya alterado por entregas.
2. Los archivos `/tmp/swarm_summit/summitN.sdf` y los archivos Xacro/launch
   deben corresponder a la auditoría H.1. El verificador comprueba sus SHA-256.
3. Generar plan NUEVO con D.9.6C.2B desde odometría actual (carpeta nueva),
   y ejecutar esta compuerta enseguida; edad máxima por defecto: 120 s de Gazebo.

```
python3 scripts/d96c2b_live_pipeline.py \
  --scenario scripts/resultados/d8/seed_100_run_007/scenario.json \
  --stage pickup1 \
  --accept-namespaced-world-odom \
  --output scripts/resultados/d9_live/seed_100_pickup1_003

python3 scripts/d96c2h2a_safety_gate.py \
  --run scripts/resultados/d9_live/seed_100_pickup1_003 \
  --stage pickup1 \
  --config-report scripts/resultados/d9_live/d96c2h1_configuration_report.json \
  --observe-seconds 5
```

Si ya existe `seed_100_pickup1_003`, use otro sufijo no existente.
Si el sim está cerrado, deberá arrancarlo antes de planificar; use la misma
configuración de escenario y mundo previamente verificada. En caso de volver a
construir SDF o alterar Xacro/launch, repita la auditoría H.1.

Salida: `scripts/resultados/d9_live/seed_100_pickup1_003/d96c2h2a_safety/pickup1_safety_report.json`

Estados: `PASS_INERT_SAFETY_PRECHECK` o `REJECTED` (con `reasons`).

## Qué inspecciona

- SHA-256 de SDF, Xacro, launch, reporte dinámico y controles;
- resultado del preflight D.9.5, cotas de velocidad 1.20 m/s,
  aceleración 1.00 m/s² y separación certificada ≥1.15 m;
- reloj sim sin retroceso y captura original ≤120 s;
- odometría reciente, stamp de la odometría coherente con `/clock` (≤250 ms),
  velocidad inicial ≤0.03 m/s, error de inicio ≤0.05 m;
- distancia real entre robots ≥1.15 m;
- presencia de *publicadores* ROS de velocidad en `/summitN/cmd_vel`,
  `/model/summitN/cmd_vel`, `/cmd_vel` o equivalentes `cmd_vel_stamped`.

La comprobación de publicadores es una **instantánea de descubrimiento DDS,
no un bloqueo exclusivo ni un mutex**. No detecta necesariamente publicadores
nativos de Gazebo fuera del grafo ROS. Si hay otro controlador, rechaza.

Un resultado `PASS_INERT_SAFETY_PRECHECK` **NO AUTORIZA** `/cmd_vel`, ni valida
el seguimiento, parada de emergencia, latencia o colisiones dinámicas.
Antes de movimiento hay que implementar un único propietario de cmd_vel,
watchdog y paro real, supervisión continua, prueba de control de bajo riesgo,
además de revalidar la posición y el plan al instante de ejecución.
