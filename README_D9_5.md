# SwarmBots D.9.5 — Protocolo independiente: preflight de SOLO LECTURA

Este parche agrega **dos archivos nuevos** y no modifica los controladores, `timed_bezier.v1`, launch ni la misión D.8.

## Instalación

Fuera de Docker, desde la raíz del repositorio `~/swarmbots`:

```bash
unzip -o ~/Descargas/swarm_D9_5_preflight_solo_lectura.zip -d ~/swarmbots
```

Dentro de Docker, desde `~/swarm_ws`:

```bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/tests/test_d95_preflight.py
python3 scripts/d95_protocol_preflight.py --run scripts/resultados/d8/seed_100_run_007
```

La corrida debe contener `d92_offline_report.json` y los archivos NPZ/informes de D.9.4 en `d94_conversion`.

**Opcional**, con Gazebo ya iniciado y ROS 2 sourced: observar odometría (sin publicar):

```bash
python3 scripts/d95_protocol_preflight.py --run scripts/resultados/d8/seed_100_run_007 --ros-observe --ros-seconds 5
```

## Resultados

`seed_100_run_007/d95_preflight/d95_summary.json` y un JSON por etapa con nombre `*_INDEPENDENT_TIMED_BEZIER_PREVIEW_ONLY.json`.

Se revisan: integridad SHA-256 de entradas, cantidad/duración de segmentos por robot, geometría/derivadas y continuidad C2 entre tramos, velocidad/accel mediante envolventes conservadoras, parada final y separación conjunta sobre el horizonte hasta que termine el último robot (incluyendo esperas). El resultado es `PASS_READ_ONLY` o `REJECT`.

## Limitaciones

- `independent_timed_bezier.preview.v1` es un **manifiesto de vista previa**, NO una versión ejecutable ni una sustitución de `timed_bezier.v1`.
- No se publica jamás en `hybrid_trajectory/command` ni en `cmd_vel`.
- Las trayectorias Bézier convertidas siguen guardadas en los NPZ no ejecutables de D.9.4; este manifiesto registra duraciones y hashes de entradas, no transfiere polígonos como comandos.
- El error continuo de aceleración entre la referencia suave y la aproximación aún no está formalmente acotado. No se ha evaluado la robustez al error de seguimiento real.
- La prueba ROS opcional observa mensajes `/summit0/odom`, `/summit1/odom`, `/summit2/odom`. Si alguno no llega, revisa que el simulador haya iniciado y el namespace correcto. No constituye una aprobación para mover robots.

**Siguiente etapa**: diseñar un `timed_bezier.v2` formal y un intérprete protegido, certificar el comportamiento efectivo bajo temporización individual, probar primero sin comandos y después en simulación controlada. No intentar enviar los archivos D.9.5 al ejecutor D.8.
