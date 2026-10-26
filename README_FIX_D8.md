# D.8 — Corrección de serialización Path

Se corrigen dos puntos en `run_swarm_seed_d8.py`:

1. Las listas `launch_cmd` y `mission_cmd` se convierten a cadenas antes de guardarlas como JSON en `commands.json`.
2. El mensaje de consola del lanzamiento de Gazebo también convierte rutas `Path` a cadenas antes de usar `join`.

No cambia el generador de escenario, CBS, Hybrid V2 ni el coordinador D.7.

## Instalación

Desde Ubuntu (ajustar ruta de Descargas según ubicación del ZIP):

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_hybrid_v2_D8_fix_paths.zip -d ~/swarmbots
```

Dentro de Docker:

```bash
cd ~/swarm_ws
python3 -m py_compile scripts/run_swarm_seed_d8.py
python3 scripts/run_swarm_seed_d8.py --seed 42 --auto-confirm --execute
```

Se creará automáticamente una carpeta nueva. La anterior `seed_42_run_003` debe conservarse como registro del intento fallido.

Antes de ejecutar la misión confirme que no hay Gazebo previo activo y revise el mundo/seguridad durante el arranque. El parche supera la comprobación de sintaxis local, pero requiere validación integrada en el ROS/Gazebo del usuario.
