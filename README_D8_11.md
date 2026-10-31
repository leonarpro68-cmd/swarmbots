# D.8.11 — Grabación de odometría y comparación de cinemática

Integra un grabador **pasivo** en el coordinador D.7, alrededor de cada invocación de `run_hybrid_v2_stage_ros.py --execute`. No altera control, certificados, márgenes, velocidades máximas, GPS, Gazebo ni algoritmos.

## Instalar

Fuera de Docker, desde `~/swarmbots`:

```bash
unzip -o ~/Descargas/swarm_hybrid_v2_D8_11_mediciones_odom.zip -d ~/swarmbots
```

El parche sobrescribe `swarm_ws/scripts/run_hybrid_v2_mission_d7.py` (basado en D.8.5) y agrega `record_d8_odom.py` y `plot_d8_measured.py`. Hacer respaldo si existen cambios posteriores a D.8.5 en el coordinador. Dentro de Docker, `cd ~/swarm_ws; python3 -m py_compile scripts/run_hybrid_v2_mission_d7.py scripts/record_d8_odom.py scripts/plot_d8_measured.py`. No requiere `colcon build`.

## Ejecutar

Ejemplo:

```bash
python3 scripts/run_swarm_seed_d8.py --seed 100 --auto-confirm --execute
```

La grabación está activada por defecto. Cada etapa genera `mission/<stage>/odom_record/summit0_odom.csv` (análogamente para summit1 y summit2), `recorder.log`, `.ready`. Los CSV se generan **durante la navegación Hybrid V2 únicamente**, desde justo antes de enviar órdenes hasta que el ejecutor retorna. No incluyen los giros / RGB-D / recogidas / entregas, ni mediciones anteriores a la misión. Si el registrador no funciona la etapa no se inicia: operación conservadora.

Desactivar registro si se requiere usando `--no-record-odom` **al ejecutar `run_hybrid_v2_mission_d7.py` directamente** (D.8 no transmite esa bandera por sí mismo).

## Graficar después de misión completa

```bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/plot_d8_measured.py --run scripts/resultados/d8/seed_100_run_XXX
```

Genera `mission_hybrid_v2_comparacion_odom.png`, cuatro filas (etapas) × dos columnas (rapidez y módulo de aceleración estimada), trazos discontinuos planificados y continuos medidos. La sincronización **por el primer movimiento** es aproximada: el tramo inicial de espera del protocolo y pequeños desfases quedan fuera; no interpreta diferencias horizontales como retardo exacto del controlador.

En los CSV se guardan marcas de tiempo ROS, recepción monotónica, poses y componentes de velocidad base y mundo. La velocidad medida deriva del mensaje `nav_msgs/Odometry.twist.twist`, no del `cmd_vel`. Para la comparación de magnitudes se usa velocidad plana; la aceleración es derivada numérica de velocidad mundo después de media móvil de 7 muestras. No es aceleración inercial medida por IMU y resulta sensible a jitter, pausas y latencia.

## Limitaciones

No puede reconstruirse odometría histórica de misiones anteriores. Las gráficas anteriores son *referencias matemáticas* derivadas de Bézier; las mediciones futuras pueden tener diferencias debidas a masa/inercia, control discreto, saturación, agarre, rozamiento, latencias, etc. No se puede atribuir causalidad solo con las curvas. Un objeto puntual con masa despreciable también necesita una ley de control: no sigue la referencia automáticamente.
