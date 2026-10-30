# D8 — Semilla → escenario → Gazebo → misión D7.3 (una terminal)

**Requisitos**: mantener en el workspace los scripts de SwarmBots y el coordinador D7.3 ya validados, ROS 2 Jazzy + Gazebo Harmonic, bridge de reloj y `swarm_worlds` instalados. Esta entrega solo agrega `swarm_ws/scripts/run_swarm_seed_d8.py`; no sobrescribe D7 ni controladores.

## Instalación

Desde el HOST (suponiendo carpeta compartida `~/swarmbots`):

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_hybrid_v2_D8_semilla_un_comando.zip -d .
```

Dentro de Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m py_compile scripts/run_swarm_seed_d8.py
```

## Prueba sin arrancar Gazebo

```bash
python3 scripts/run_swarm_seed_d8.py --seed 42
```

## Iniciar escenario + simulación + misión en UNA terminal

**Cierra antes otras simulaciones Gazebo y misiones activas.**

```bash
python3 scripts/run_swarm_seed_d8.py --seed 42 --auto-confirm --execute
```

El script:

1. Genera el JSON con `escenario_aleatorio_swarm.py` usando `--seed`.
2. Genera un SDF nuevo con `swarm_cbs_grasp_prepare.py` desde el mundo original.
3. Arranca el launch `swarm_cbs_grasp_base.launch.py` con seed, scenario, world_sdf, hybrid_enabled y hybrid_max_speed.
4. Comprueba `/clock`; en caso necesario arranca un puente de reloj.
5. Espera mensajes reales de odometría de los tres Summit, nodos de navegación y manipulación y los nueve objetos en Gazebo.
6. Inicia D7.3 con `--auto-confirm`, que pide una autorización inicial SI y después gestiona navegación, agarres y oleadas de entrega.
7. Al finalizar o fallar, cierra los procesos Gazebo y puente que lanzó; conserva archivos y logs.

La carpeta se crea automáticamente como `scripts/resultados/d8/seed_42_run_001`, luego `_002`, etc.; contiene `scenario.json`, `world.sdf`, `gazebo_launch.log`, `commands.json` y `mission/`.

**Importante**: este automatizador NO certifica automáticamente la identidad espacial odom↔Gazebo, el volumen de las cargas ni la seguridad continua durante aproximaciones locales. El operador debe verificar el escenario antes de autorizar `SI`. Los límites del planificador siguen siendo 1.20 m/s y 1.00 m/s²; el radio experimental sigue siendo 0.55 m. La seguridad física en otras semillas no está validada aún. Detener la misión ante cualquier comportamiento peligroso.

Si `ros2 launch` rechaza `hybrid_enabled` o `hybrid_max_speed`, significa que el launch instalado difiere del validado en tu entorno: actualiza la instalación desde el proyecto existente; no elimines parámetros sin revisar su efecto.

Si los mensajes `/clock` o las odometrías no aparecen, consulta `gazebo_launch.log` y comprueba `ros2 topic info /clock -v` y `ros2 node list`. El script no reinicia Gazebo automáticamente tras errores para evitar mezclar estados físicos.
