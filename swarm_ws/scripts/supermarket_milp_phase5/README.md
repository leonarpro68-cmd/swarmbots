# SwarmBots — Fase 5 · Integración MILP–CBS–Hybrid V2–V3 con D8

**Versión de preparación y preflight. No inicia Gazebo ni ejecuta movimientos.**

La misión fija de 3 Summit/6 basuritas ya fue validada. Este paquete sirve para preparar **una misión MILP distinta** (semilla 100): verifica las cuatro trayectorias nuevas, cruza su asignación con CBS, genera los cuatro archivos de tareas físicas D8 y examina las aproximaciones locales. **No reutilices los reportes físicos ni NPZ de la misión fija.**

## Lo que verifica

- Misma semilla 100, escenario `supermarket` y hash del archivo de escenario registrado por CBS.
- Exactamente seis basuritas diferentes, dos por Summit, en cuatro etapas encadenadas y sin discontinuidad.
- Para cada NPZ: hash SHA256 idéntico al reloj V3; controles finitos, dimensiones `(K,6,2)`, duración positiva, inicios/finales que coinciden con CBS.
- Reporte Hybrid V2 con cinco refinamientos y certificado continuo aprobado; reloj común V3 con límites 2 m/s y 2 m/s².
- Distancias y segmento ideal local frente a estanterías, otras basuritas pendientes y depósitos distintos del destino.
- Exportación de `assignment_pickup1_MILP.json`, `assignment_delivery1_MILP.json`, `assignment_pickup2_MILP.json`, `assignment_delivery2_MILP.json`, y `scenario_for_original_D8_seed100_MILP.json`.

### Restricciones D8 realmente implementadas en los scripts originales

- `prueba_alinear_agarrar_cbs.py`: 0.55–2.20 m para pickup.
- `prueba_entrega_local_cbs.py`: posición inicial entre 0.90 y 1.80 m del depósito, parada objetivo a 0.80 m.
- La distancia nominal **no valida** cámara, orientación, pinza ni la geometría de movimiento real de RGB-D.

La prueba de referencia con el modelo MILP corregido detectó 12/12 distancias dentro del rango. Advertencias: `pickup1` summit1 margen 0.087 m respecto al límite de 2.2; `delivery1` summit2 y `delivery2` summit0, margen **0.032 m** respecto al límite de 1.8. **No aumentar los límites para sortear un rechazo de odometría.** Debe corregirse la meta y regenerarse CBS/Hybrid/V3 si se necesita más margen.

## Instalación (Ubuntu FUERA de Docker)

Descomprimir el archivo `supermarket_milp_fase5_preflight_v1.zip` en `~/Descargas` y ejecutar:

```bash
bash ~/Descargas/supermarket_milp_fase5_preflight_v1/install_on_host.sh \
  ~/swarmbots/swarm_ws
```

La instalación crea **solo** `scripts/supermarket_milp_phase5/`. No recompila, no toca `sim_summit.launch.py`, `supermarket_support.launch.py`, algoritmos ni resultados antiguos.

## Pruebas y preparación (Docker, SIN GAZEBO)

```bash
cd ~/swarm_ws
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 -m unittest discover -s scripts/supermarket_milp_phase5/tests -v
```

Luego, con el **CBS corregido** y tus cuatro resultados fase 4:

```bash
CBS=/tmp/milp_cbs_multigoal_corr_live_v1.json
SC=~/swarm_ws/config/supermarket/seed_100.json
F4=~/swarm_ws/scripts/supermarket_milp_hybrid_v2_v3/results/milp_100_phase4_v1
OUT=~/swarm_ws/scripts/supermarket_milp_phase5/results/milp_seed100_01

python3 scripts/supermarket_milp_phase5/prepare_phase5.py \
  --cbs "$CBS" --scenario "$SC" --results "$F4" --output-dir "$OUT"
```

El comando **rechaza sobrescrituras** de un directorio de salida no vacío. Si está todo conforme, debe imprimir `PASS_STATIC_PRECHECK_NOT_EXECUTABLE`, seguidamente las 12 filas D8. Si hay errores, la salida JSON contendrá la explicación y **no se crearán asignaciones**.

Inspeccionar:

```bash
python3 -m json.tool "$OUT/phase5_static_preflight.json"
cat "$OUT/assignment_pickup1_MILP.json"
```

## Comandos de navegación (solo generación de texto, NO ejecución)

```bash
python3 scripts/supermarket_milp_phase5/print_navigation_commands.py \
  --prepared "$OUT" --stage pickup1 --run-id milp_seed100_01
```

El script muestra comandos para el alineador D9.9.7 y coordinador D9.9.13 con el **NPZ correcto de MILP** y SHA fijo. No ejecuta ninguno. Para posteriores etapas cambiar `--stage`. Si reinicias Gazebo debes comenzar `pickup1`, no ejecutar `pickup2` directamente.

## Preflight D8 en vivo, solo DESPUÉS de navegación realmente completada

Cuando esté activo Gazebo, haya terminado la navegación de la etapa y se haya inspeccionado el estado de la carga, se puede ejecutar el dry-run nativo. Ejemplo para la primera basurita MILP de Summit 0:

```bash
python3 scripts/supermarket_milp_phase5/live_physical_dry.py \
  --prepared "$OUT" --stage pickup1 --robot summit0 \
  --navigation-report scripts/resultados/milp_seed100_01_pickup1_REAL.json \
  --confirm-current-session \
  --output /tmp/milp_seed100_01_pickup1_summit0_DRY.json
```

Para `delivery1`/`delivery2` se exige además `--confirm-loaded` **solo después de verificar visualmente la caja en el gripper**. El preflight valida informe de navegación, odometría, estado de robots y ejecuta el modo **sin movimiento** del script D8 original. No tiene opción `--execute` a propósito. Tampoco confirma que la caja real coincida con el JSON en el momento de la descarga.

## Límites de esta entrega

**El resultado `PASS_STATIC_PRECHECK_NOT_EXECUTABLE` no autoriza ejecución en Gazebo.** La decisión final de mover robots requiere, por separado, un preflight ROS para los tres Summit (TF, odometría, único publicador `/cmd_vel`), prueba de alineación D9.9.7 y dry-run D9.9.13, además de inspección supervisada de las maniobras físicas D8. Si hay discrepancias de distancia, no modificar seguridad para lograr un PASS.

El siguiente paso es validar el arranque de la primera etapa MILP en una **sesión Gazebo nueva**, comenzando con comprobación y alineación. Conservamos el lanzamiento `supermarket_support.launch.py` ya aprobado.
