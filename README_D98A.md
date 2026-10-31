# D.9.8A — puente de navegación D9.2 a misión D6/D8 (solo lectura)

Este parche **no ejecuta** agarre, no actualiza un registro D6 y no publica `/cmd_vel`. Verifica que la navegación D9.7C.1 terminó con normalidad, que su relay detuvo los robots, que los controles convertidos no cambiaron y que los identificadores de objetos provienen del plan D8 de la misma etapa. NO equivale a una misión de recogida.

## Instalación

Descomprimir en `~/swarmbots`; reemplazar solo los archivos ubicados bajo `swarm_ws/scripts/`. Dentro de Docker:

```bash
cd ~/swarm_ws
python3 -m unittest discover -s scripts/tests -p test_d98a_handoff.py -v
```

## Validar el pickup1 completado

```bash
python3 scripts/d98a_mission_handoff.py \
  --run scripts/resultados/d9_live/seed_100_pickup1_009 \
  --stage pickup1 \
  --relay-report scripts/resultados/d9_live/d97c1_relay_full_002.json
```

Esta comprobación busca `...009/live/pickup1_live_report.json`, `...009/d96c2b_report.json`, el manifiesto de controles convertidos, el informe del seguidor y el del relay. El resultado se guarda en `...009/d98a/pickup1_handoff.json`.

`READY_FOR_PHYSICAL_CONFIRMATION_ONLY` solo significa que podemos pasar a la fase de confirmación física, no que la recogida ya ocurrió. Si alguna dependencia falta, devuelve `BLOCKED_HANDOFF` con motivos. No reutilizar un informe de relay perteneciente a otra ejecución; el script **no puede verificar criptográficamente** su vinculación porque ese informe no lleva ID de sesión compartida.

## Siguiente integración (pendiente de implementar en D9.8B)

El `grasp_manager.py` existente atiende `/<robot>/gripper/grasp` y `/<robot>/gripper/release` (`std_msgs/Empty`), y reporta resultados mediante `/<robot>/gripper/result` y `/<robot>/gripper/release_result` (`std_msgs/Int8`). Hay que correlacionar físicamente objeto previsto, resultado de pinza y objeto efectivamente unido/liberado. Solo entonces se puede actualizar el ledger D6 con un informe certificado de la **misma** etapa y confirmación del operador. Las operaciones de recogida y entrega de D8 incluyen barreras entre etapas y no deben sustituirse por un simple `FINISHED_TRAJECTORY_NOT_MISSION`.

**Limitaciones**: D9.7C.1 del parche actual acepta únicamente `pickup1`; otras etapas todavía necesitan integración del ejecutor y el contrato de sesión, así como un watchdog que pueda funcionar a través de cada fase. La comparación real de D8 con D9.2 requiere tiempos homogéneos y registro de velocidades/aceleraciones medidos. Gazebo no tiene un failsafe demostrado ante la caída del proceso relay.
