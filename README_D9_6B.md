# D.9.6B — lector de `/clock` ROS 2 sin movimiento

Este parche añade `scripts/d96b_clock_readonly.py` y pruebas de regresión. **No modifica D.8**, no crea publicadores ROS, no envía `/cmd_vel`, no simula que los robots se muevan y no habilita el protocolo `timed_bezier.v1`. Es un experimento para reproducir **referencias** D.9.2/D.9.4 con el tiempo real de simulación.

## Instalación

Fuera del Docker:

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_D9_6B_reloj_ros_solo_lectura.zip -d ~/swarmbots
```

Dentro del Docker, con ROS 2 Jazzy y el workspace cargados:

```bash
cd ~/swarm_ws
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/tests/test_d96b_clock.py
```

## Uso (Gazebo debe estar activo y publicando `/clock`)

**Dejar Gazebo activo y no ejecutar la misión D.8 simultáneamente.** En una terminal dentro del Docker:

```bash
ros2 topic list | grep clock
ros2 topic echo /clock --once
python3 scripts/d96b_clock_readonly.py --run scripts/resultados/d8/seed_100_run_007 --stage pickup1
```

Para otras etapas cambiar `--stage` por `delivery1`, `pickup2` o `delivery2`; cada invocación inicia una nueva reproducción **virtual** desde el primer mensaje de reloj. Requiere un entorno ROS 2 configurado en Docker; no necesita robots moviéndose, pero sí Gazebo activo.

## Salida

- `seed_100_run_007/d96b_clock_readonly/pickup1_D96B_CLOCK_READ_ONLY.csv`: referencias simultáneas de posición/velocidad/aceleración para cada Summit.
- `seed_100_run_007/d96b_clock_readonly/pickup1_D96B_report.json`: `COMPLETE_READ_ONLY` indica que transcurrió el horizonte simulado con reloj válido.
- `CLOCK_NOT_AVAILABLE`: no llegó `/clock`.
- `CLOCK_STALLED_OR_GAZEBO_PAUSED`: faltan pulsos de reloj (pausa Gazebo o problema de conexión).
- `CLOCK_REWIND_ABORT`: el reloj retrocedió, por ejemplo tras resetear simulación.

No se comprueba odometría ni control: no es un arranque certificado ni una prueba de seguimiento. Usa el primer pulso positivo como origen temporal, no una orden sincronizada `start`. Los resultados de seguridad de D.9.5 aún no autorizan movimiento hasta contar con un seguidor y una cadena de validación ejecutable.
