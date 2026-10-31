# D.9.6C.1b — corrección de doble tolerancia (solo modo sombra)

## Causa
El primer punto de la referencia Bézier convertida `pickup1` de Summit 0 está a 0.122 m de `scenario.json`. El script anterior exigía que ese **desfase geométrico de referencia** fuera <= 0.10 m, usando la misma tolerancia destinada a comparar la **odometría medida** con la posición del escenario. Son comprobaciones independientes y no debe interpretarse el desfase de planificación como fallo de ROS o del simulador.

## Cambios
- **Odometría real vs escenario:** sigue limitada a **0.10 m**, no se relaja.
- **Primer punto de la trayectoria vs escenario:** en modo `--gazebo-aligned-shadow` admite hasta **0.15 m** por defecto, rechazando diferencias mayores; límite absoluto de argumento <= 0.20 m. El modo normal mantiene el rechazo estricto a 0.10 m.
- `--inspect-starts` imprime las posiciones de escenario y referencia y el desfase por robot, sin cargar `rclpy` ni conectarse a Gazebo.
- Reporte incluye `reference_scenario_start_offsets_m`, `reference_start_details`, `reference_start_tolerance_m` y sigue mostrando `initial_alignment_errors_m` (odometría vs escenario).
- No modifica las curvas D.9.4, el manifiesto D.9.5, D.8 ni ROS TF. No crea publishers ni envía `/cmd_vel`.

**Esto no valida equivalencia entre `map` y `summitN/odom`, especialmente la rotación entre marcos. Solo sirve para evaluar cálculos hipotéticos en modo sombra. No habilitar movimiento con esta bandera.**

## Instalación fuera del contenedor Docker
```bash
cd ~/swarmbots
cp swarm_ws/scripts/d96c1_shadow.py swarm_ws/scripts/d96c1_shadow.py.bak_pre_startfix
unzip -o ~/Descargas/swarm_D9_6C_1b_fix_inicio_referencia.zip -d ~/swarmbots
```
El Docker debe montar `~/swarmbots/swarm_ws` como `~/swarm_ws`, igual que las etapas anteriores.

## Tests y diagnóstico dentro de Docker
```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 -m py_compile scripts/d96c1_shadow.py
python3 scripts/tests/test_d96c1b_reference_offset.py
python3 scripts/tests/test_d96c1b_alignment.py
python3 scripts/d96c1_shadow.py --run scripts/resultados/d8/seed_100_run_007 --stage pickup1 --inspect-starts
```

## Prueba sombra real (requiere Gazebo con robots y bridge /clock + 3 odometrías)
```bash
python3 scripts/d96c1_shadow.py --run scripts/resultados/d8/seed_100_run_007 --stage pickup1 --gazebo-aligned-shadow
```
Salida: `scripts/resultados/d8/seed_100_run_007/d96c1_shadow/pickup1_shadow_report.json`. En el JSON, `initial_alignment_ok` debe ser `true`, `reference_scenario_start_offsets_m` incluirá el desfase previsto, `shadow_status_counts` puede incluir `BLOCKED_TRACKING_ERROR` porque robots están quietos, y `commands_sent` debe ser 0.

Si cualquier inicio Bézier dista más de 0.15 m, el script bloquea e informa las distancias. Inspeccionar el escenario y el plan antes de cualquier ajuste; NO elevar los límites para encubrir un error de planificación.
