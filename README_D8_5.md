# D8.5: cierre opcional de Gazebo y confirmación inicial opcional

Solo sustituir los dos scripts incluidos. No modifica CBS, Hybrid V2, el launch, ni grasp_manager.

## Instalación
Desde la carpeta host que contiene `swarm_ws/`:

```bash
unzip -o swarm_hybrid_v2_D8_5_control_cierre_confirmacion.zip
```

Desde Docker `~/swarm_ws`:

```bash
python3 -m py_compile scripts/run_swarm_seed_d8.py scripts/run_hybrid_v2_mission_d7.py
```

## Uso

Sin SI y Gazebo abierto al finalizar (por defecto):

```bash
python3 scripts/run_swarm_seed_d8.py --seed 123 --auto-confirm --execute
```

Cerrar automáticamente al terminar:

```bash
python3 scripts/run_swarm_seed_d8.py --seed 42 --auto-confirm --close-gazebo --execute
```

Solicitar SI inicial:

```bash
python3 scripts/run_swarm_seed_d8.py --seed 42 --auto-confirm --require-start-confirmation --execute
```

Mantener Gazebo abierto explícitamente:

```bash
python3 scripts/run_swarm_seed_d8.py --seed 42 --auto-confirm --keep-gazebo --execute
```

Con `--keep-gazebo`, el proceso D8 permanece abierto hasta que el operador pulse Ctrl+C; entonces se cierran Gazebo y sus puentes. Ante errores o fallos de misión, se cierran en cualquier modo. El modo sin SI elimina la aprobación humana: antes de utilizarlo el operador debe haber comprobado la escena y seguridad de operación; los chequeos `READY` y los preflights no certifican las aproximaciones locales.
