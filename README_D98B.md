# D.9.8B — preflight de agarre físico (primera barrera, solo lectura)

El `grasp_manager` real publica `/summitN/gripper/result` (`std_msgs/Int8`, 1 o -1), pero no informa el nombre del objeto. Antes de enviar `/gripper/grasp` es imprescindible identificar el objeto más próximo a la punta de cada pinza, ya que el manager elige **el más cercano de todas las basuras del mundo**, no necesariamente el asignado por CBS.

`d98b_grasp_preflight.py` consulta `gz topic -e /world/<world>/pose/info`, odometrías ROS `/summitN/odom`, estima las puntas de pinza con `palm_forward=0.45` y compara distancias con la asignación D8 de `pickup1` en el handoff D98A. Comprueba unicidad, inmovilidad (<0.03m/s), radio <=0.45m y margen de ambigüedad >=0.07m. **No crea publicadores, no mueve robots, no intenta agarrar ni actualiza el ledger.**

Instalar en host: `unzip -o ~/Descargas/swarm_D98B_preflight_agarre.zip -d ~/swarmbots`

En Docker (`cd ~/swarm_ws`):
```
python3 -m unittest discover -s scripts/tests -p test_d98b_grasp_preflight.py -v

gz topic -l | grep '/pose/info'
python3 scripts/d98b_grasp_preflight.py \
  --handoff scripts/resultados/d9_live/seed_100_pickup1_009/d98a/pickup1_handoff.json \
  --world world_demo
```
**Cambiar `world_demo` por el nombre real mostrado en `/world/<nombre>/pose/info`.** El informe se genera en `.../009/d98b/pickup1_grasp_preflight.json`.

`READY_FOR_MANUAL_GRASP_TEST_ONLY` solo significa que las tres puntas se encuentran cerca de los tres objetos correctos, no que se agarraron. En el siguiente subpaso se integrarán ROS Int8 ACK + verificación Gazebo del detachable joint + escritura transaccional D6, *con la sesión actual*.

Si falla `BLOCKED`, no disparar agarres. Este verificador no demuestra que los robots sigan cerca del mismo objeto tras transcurrir tiempo ni sustituye una comprobación final justo antes de enviar comandos. Si el escenario ha cambiado o se reinició Gazebo, volver a planificar pickup1 y ejecutar D9.2.
