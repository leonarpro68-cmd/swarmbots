# D.9.6A — Seguidor de referencia de solo lectura (sin comandos ROS)

Este parche **no sustituye `timed_bezier.v1`, no inicia Gazebo y no publica comandos**.

## Contenido
- `swarm_ws/scripts/d96a_readonly_follower.py`: comprueba hash SHA256 de los tres insumos registrados por D9.5, lee los controles y las duraciones individuales de cada Summit, y evalúa posicion, velocidad y aceleración bajo un reloj de misión compartido `t_s`.
- `swarm_ws/scripts/tests/test_d96a_readonly.py`: pruebas del evaluador, espera al completar, bloqueo por hash y rechazo de manifiesto ejecutable.

## Instalación
Desde el host:
```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_D9_6A_lector_solo_lectura.zip -d ~/swarmbots
```
Dentro del contenedor (D9/D9.1/D9.2/D9.4/D9.5 deben estar previamente instalados):
```bash
cd ~/swarm_ws
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/tests/test_d96a_readonly.py
python3 scripts/d96a_readonly_follower.py --run scripts/resultados/d8/seed_100_run_007
```
Opcional, solo una etapa:
```bash
python3 scripts/d96a_readonly_follower.py --run scripts/resultados/d8/seed_100_run_007 --stage pickup2 --step 0.02
```

## Resultados
`seed_100_run_007/d96a_readonly/d96a_summary.json` y un CSV mas un JSON por etapa.

Los CSV exportan `t_s`, `x_m`, `y_m`, `vx_m_s`, `vy_m_s`, `ax_m_s2`, `ay_m_s2`, `done`, evaluados simultáneamente para los tres robots. Tras finalizar un robot, mantiene su pose final con velocidad y aceleración nulas.

## Límites
- El reloj es simulado en análisis offline, **no se sincroniza todavía con `/clock` de ROS**.
- No crea ningún nodo, publicador ni comando de movimiento. Este paso prepara la capa de evaluación que será utilizada en el lector ROS read-only futuro.
- La validez depende del preflight D9.5 existente y de los hashes de sus insumos; no constituye autorización de ejecución.
- No certifica error continuo de aceleración entre D9.2 y D9.4.
- El seguidor de control real y sincronización ROS son tareas posteriores.
