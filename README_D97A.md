# D.9.7A — Núcleo del seguidor holonómico de tiempos independientes

Esta entrega construye el núcleo matemático del ejecutor, **sin publicar comandos ROS**. D.8 permanece sin modificaciones.

Contenido:
- `swarm_ws/scripts/d97_executor_core.py`: controlador cinemático, saturación de velocidad 1.2 m/s y cambio de comando 1.0 m/s², verificación conjunta de marcos/seguimiento/separación y lectura de trayectorias certificadas D.9.5.
- `swarm_ws/scripts/tests/test_d97_executor_core.py`: 8 pruebas puras sin Gazebo ni ROS.

## Instalación
En el host: `cd ~/swarmbots && unzip -o ~/Descargas/swarm_D97A_nucleo_ejecutor.zip -d ~/swarmbots`

Dentro de Docker:
```
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/scripts:$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m unittest discover -s scripts/tests -p test_d97_executor_core.py -v
python3 scripts/d97_executor_core.py --run scripts/resultados/d9_live/seed_100_pickup1_005 --stage pickup1
```
Si el plan vigente está en otra carpeta, sustituir el argumento `--run`.

El informe queda en `<run>/d97_core/pickup1_core_preview.json`.

## Alcance y restricciones
El modo preview presupone seguimiento perfecto y yaw=0 para probar únicamente las referencias temporales y el núcleo de cálculo. **NO demuestra seguimiento físico**, ni seguridad de actuación. La limitación de aceleración se aplica al cambio de comando en marco del robot; para pruebas con rotación debe implementarse vigilancia de velocidad/aceleración medidas en el marco global. La comprobación de separación es de poses observadas y requiere odometrías sincrónicas para uso real.

El ejecutor ROS activo, la confirmación de los tópicos Twist efectivos, un watchdog independiente de pérdida de comandos, la autoridad exclusiva de publicación y la prueba de detención en Gazebo son trabajos siguientes imprescindibles. No ejecutar la misión D.9.2 todavía. No usar este núcleo como actuador directo.
