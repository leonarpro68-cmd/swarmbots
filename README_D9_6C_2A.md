# D.9.6C.2A — Puerta de origen vivo de solo lectura

**NO ES CONTROLADOR EJECUTABLE.** No modifica D.8, no publica TF ni `/cmd_vel`. Primer paso para integrar la temporalización independiente con la captura de odometría actual.

## Qué hace

- Carga un D9.5 `PASS_READ_ONLY` con verificación de integridad.
- Lee la odometría original que D8 capturó al crear el plan (`*_live_report.json`).
- Comprueba que el primer control Bézier coincide con la odometría original.
- Lee UNA instantánea actual de `/summit0/odom`, `/summit1/odom`, `/summit2/odom` y compara con el origen del plan archivado.
- Rechaza reutilización de planes antiguos si el Summit ya no está en su punto inicial, si faltan datos o si los marcos declarados cambiaron.
- Informa siempre `execution_authorized: false`: coincidencia de coordenadas no demuestra equivalencia global TF ni seguridad de ejecución.

## Instalación

Fuera de Docker: `unzip -o ~/Descargas/swarm_D9_6C_2A_origen_vivo_solo_lectura.zip -d ~/swarmbots`

Dentro de Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/tests/test_d96c2a_live_gate.py
```

Con Gazebo y el puente de odometría funcionando (los tres Summit quietos):

```bash
python3 scripts/d96c2a_live_start_gate.py \
  --run scripts/resultados/d8/seed_100_run_007 --stage pickup1
```

Informe: `scripts/resultados/d8/seed_100_run_007/d96c2a_live_start/pickup1_live_gate.json`.

`REQUIRES_LIVE_REPLAN` es **esperable** si se reutiliza una captura antigua de D8 en una nueva simulación. NO subas `--start-tolerance` para forzar el paso. El siguiente desarrollo debe integrar la generación de trayectorias desde la captura actual y volver a calcular D9.2, D9.4 y las certificaciones antes de autorizar el seguimiento. La diferencia de 12,17 cm de Summit 0 frente a la posición nominal del escenario NO significa que el plan original fuera incorrecto: el plan comienza exactamente en la odometría que D8 capturó.
