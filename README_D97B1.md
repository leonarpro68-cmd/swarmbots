# D.9.7B.1 — prueba de transporte de comandos cero (ROS 2 Jazzy/Gazebo Harmonic)

Este parche **NO mueve robots**, no ejecuta pickup1 y no sustituye D.8.

## Alcance
- Lee `/clock` y `/summit{0,1,2}/odom`.
- Rechaza si hay publicadores en los 10 tópicos inspeccionados antes de crear los suyos.
- Al pasar `--send-zero`, publica únicamente `geometry_msgs/Twist()` (todos los campos = 0) en `/summit{0,1,2}/cmd_vel` durante 4 s.
- Vuelve a consultar publicadores durante la prueba y comprueba deriva de posición en reposo.
- Guarda un JSON. **NO certifica que Gazebo haya recibido los ceros, no dispone de watchdog independiente y NO prueba parada ante pérdida de proceso.** Su resultado no autoriza movimiento.

## Instalación
En el host:
```
cd ~/swarmbots
unzip -o ~/Descargas/swarm_D97B1_ceros_seguro.zip -d ~/swarmbots
```
En Docker:
```
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m unittest discover -s scripts/tests -p test_d97b1_zero_actuator.py -v
```

## Preparación
Mantener abierto Gazebo seed 100, tres Summit quietos, puente ROS↔Gazebo `/clock` y odometrías. No ejecutar misión D.8 ni D.9.2. Confirmar que ningún seguidor publica en `/summitN/cmd_vel` y que el bridge ROS↔Gazebo acepta ese tópico de entrada (véase sección siguiente).

## Verificación del puente real
```
for i in 0 1 2; do
  ros2 topic info /summit${i}/cmd_vel -v
  gz topic -i -t /model/summit${i}/cmd_vel
done
```
En el lanzamiento de este proyecto, `MecanumDrive` consume **Gazebo** `/model/summitN/cmd_vel`. Se necesita un puente ROS→Gazebo desde `/summitN/cmd_vel` hacia ese tópico; antes de mover robots en el futuro debe verificarse con el launch realmente utilizado. Esta prueba de ceros no demuestra esa ruta por sí sola.

## Ejecutar primero sin publicar
```
python3 scripts/d97b1_zero_actuator.py --output scripts/resultados/d9_live/d97b1_precheck.json
```
Debe salir `PASS_PREFLIGHT_NO_PUBLISHERS` (cero publicadores creados).

## Publicar exclusivamente ceros
```
python3 scripts/d97b1_zero_actuator.py --send-zero --output scripts/resultados/d9_live/d97b1_zero_report.json
```
Esperado: `PASS_ZERO_TRANSPORT_OBSERVED_NOT_FAILSAFE`, 3 publicadores y solo velocidades cero. Si hay conflicto o faltan sensores se rechaza.

**No hay frenado redundante ante un `kill -9`, pérdida de enlace o caída de Gazebo**. La siguiente implementación debe comprobar la ruta ROS→GZ y el timeout real del plugin antes de emitir cualquier velocidad distinta de cero.
