# SwarmBots — Guía de ejecución: MILP + escalado colectivo sucesivo V3

**Configuración validada:** ROS 2 Jazzy, Gazebo Harmonic, **3 Summit XL**, **6 basuras**, **3 depósitos**, semilla **100**. Etapas: `pickup1 → delivery1 → pickup2 → delivery2`.

Este README está pensado para **ejecutar** la versión D9.9.32 y reproducir el último experimento. La planificación utiliza **A* + CBS + Hybrid V2 (Bézier quínticas)**, **MILP** para elegir la asignación y el **reloj colectivo sucesivo V3** para ejecutar las trayectorias. El procedimiento fue reconstruido a partir de los scripts y manuales del proyecto. No es necesario reinstalar parches ni recalcular las 720 asignaciones para repetir una misión ya preparada.

> **Dónde ejecutar cada comando:** los comandos Git, `cp` hacia el repositorio y `docker exec` se ejecutan en **Ubuntu (host)**. Los comandos `ros2`, `colcon` y `python3 scripts/...` se ejecutan **dentro de Docker (`swarm_dev`)**, en `/home/dev/swarm_ws` (equivalente a `~/swarm_ws` para el usuario `dev`).

---

## 1. Abrir el contenedor y preparar ROS 2

**Ubuntu (host), terminal 1:**

```bash
docker exec -it swarm_dev bash
```

**Docker:**

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/scripts:$PWD/src/swarm_behavior:${PYTHONPATH:-}"
```

Si has modificado los scripts de navegación ROS o el protocolo V3 desde la última compilación, **con Gazebo apagado**, compila y vuelve a cargar el entorno:

```bash
colcon build --packages-select swarm_behavior swarm_worlds --symlink-install
source install/setup.bash
```

La planificación y la misión requieren la versión de los scripts **D9.9.29–D9.9.32** y el seguidor V3 correspondiente.

## 2. Comprobar que ya existen la biblioteca MILP y las trayectorias V3

**Docker:**

```bash
cd ~/swarm_ws

ls -l scripts/milp_library/seed_100/best_assignment.json
ls -l scripts/d996_reference/scenario.json
ls -l scripts/d996_reference/world.sdf
ls -l scripts/resultados/d9931_seed100_v3_001/manifest.json
ls -l scripts/resultados/d9931_seed100_v3_001/*_clock.json
ls -l scripts/resultados/d9931_seed100_v3_001/*.npz
```

Si los archivos anteriores están presentes, **puedes ir directamente al paso 3**. No recalcules el MILP en cada prueba. Los archivos `manifest.json` y `*_clock.json` forman parte del paquete V3: **no son opcionales**.

## 3. Preflight del paquete V3 (sin movimiento)

**Docker:**

```bash
cd ~/swarm_ws
python3 scripts/d9927_milp_preflight.py \
  --scenario scripts/d996_reference/scenario.json \
  --library-dir scripts/milp_library \
  --timed-dir scripts/resultados/d9931_seed100_v3_001 \
  --output scripts/resultados/d9931_seed100_preflight_repeticion.json
```

Comprobar:

```bash
python3 - <<'PY'
import json
p = 'scripts/resultados/d9931_seed100_preflight_repeticion.json'
r = json.load(open(p))
print('Estado:', r.get('status'))
print('Semilla:', r.get('seed'))
print('Etapas:', list(r.get('stages', {})))
PY
```

**Resultado esperado:** `MILP_V3_PREFLIGHT_VALID_OFFLINE_ONLY`, semilla `100` y las **cuatro** etapas. Este estado indica compatibilidad e integridad **offline**; no inicia Gazebo ni autoriza movimiento.

## 4. Iniciar Gazebo con el escenario de la semilla 100

Usa una **sesión limpia** de Gazebo. No reutilices un mundo en el que los robots ya hayan recogido o entregado objetos.

**Docker, terminal 1 (dejar abierta):**

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch swarm_worlds swarm_cbs_grasp_base.launch.py \
  seed:=100 \
  scenario:="$PWD/scripts/d996_reference/scenario.json" \
  world_sdf:="$PWD/scripts/d996_reference/world.sdf" \
  hybrid_enabled:=false \
  headless:=false
```

Este launch carga el escenario y los nodos de los Summit, **sin iniciar la misión de forma automática**. Espera a ver los tres robots en Gazebo.

**Ubuntu (host), terminal 2:**

```bash
docker exec -it swarm_dev bash
```

**Docker, terminal 2:**

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/scripts:$PWD/src/swarm_behavior:${PYTHONPATH:-}"

ros2 node list
ros2 topic list | grep -E 'clock|odom'
```

No inicies la ejecución física si falta `/clock`, la odometría o alguno de los tres robots.

## 5. Prueba de misión sin movimiento (dry-run)

**Docker, terminal 2:**

```bash
python3 scripts/d9922_mission_with_telemetry.py \
  --assignment-method milp \
  --milp-library-dir scripts/milp_library \
  --milp-timed-dir scripts/resultados/d9931_seed100_v3_001 \
  --reference-dir scripts/d996_reference \
  --nav-speed-limit 2.0 \
  --nav-accel-limit 2.0 \
  --output-dir scripts/resultados/milp_v3_dry_nuevo_001
```

Revisa el resultado:

```bash
python3 - <<'PY'
import json
p = 'scripts/resultados/milp_v3_dry_nuevo_001/report.json'
r = json.load(open(p))
for k in ('status', 'execute', 'mission_exit_code', 'problems'):
    print(f'{k}: {r.get(k)}')
PY
```

**Resultado esperado:** `status = DRY_RUN_DONE`, `execute = False`, `mission_exit_code = 0` y `problems = []`.

> El programa **rechaza** un directorio de salida que ya exista. Cambia `_001` a `_002`, `_003`, etc. para pruebas posteriores. Un dry-run **no** sustituye la comprobación del escenario antes de mover robots.

## 6. Ejecutar la misión física completa MILP + V3

Después de que el preflight y el dry-run sean correctos, y con **Gazebo limpio y los tres Summit en sus posiciones iniciales**:

**Docker, terminal 2:**

```bash
python3 scripts/d9922_mission_with_telemetry.py \
  --assignment-method milp \
  --milp-library-dir scripts/milp_library \
  --milp-timed-dir scripts/resultados/d9931_seed100_v3_001 \
  --reference-dir scripts/d996_reference \
  --nav-speed-limit 2.0 \
  --nav-accel-limit 2.0 \
  --execute \
  --confirm-runtime-v2 \
  --confirm-cargo-inspected \
  --enable-successive-execution \
  --output-dir scripts/resultados/milp_v3_real_nuevo_001
```

La misión comprende, en orden: navegación hacia las tres primeras basuras, **recogida física paralela**, navegación hacia los depósitos, entrega física por oleadas; se repite para las tres basuras restantes. El sistema aplica MILP a la asignación y el reloj sucesivo a la navegación de cada etapa.

**Importante:** en V3 **NO** añadir `--auto-time-scale` ni usar `--time-scale` distinto de `1`. El reloj sucesivo ya define los tiempos efectivos y la segunda transformación temporal está bloqueada en el supervisor.

## 7. Verificar el resultado y consultar gráficas

**Docker, terminal 2:**

```bash
python3 - <<'PY'
import json
p = 'scripts/resultados/milp_v3_real_nuevo_001/report.json'
r = json.load(open(p))
for k in ('status', 'mission_status', 'mission_exit_code',
          'stage_count_with_telemetry', 'problems'):
    print(f'{k}: {r.get(k)}')
PY
```

Estados observados en la ejecución V3 validada de la semilla 100:

```text
status = DONE_MISSION_WITH_TELEMETRY_AND_METRICS
mission_status = DONE_ALL_FOUR_PHYSICAL_STAGES_ACK
mission_exit_code = 0
stage_count_with_telemetry = 4
problems = []
```

Productos principales (en `scripts/resultados/milp_v3_real_nuevo_001/`):

| Archivo | Contenido |
|---|---|
| `report.json` | Estado final de la misión, telemetría y métricas |
| `mission.log` | Registro del ejecutor físico |
| `mision_fisica/report.json` | Resultado de las cuatro etapas físicas |
| `mision_fisica/pickup1_navigation.json` | Duración por robot y parámetros del reloj de `pickup1` |
| `odometria.jsonl` | Mediciones registradas de odometría |
| `stage_windows.json` | Ventanas temporales de las cuatro etapas |
| `velocidad_aceleracion_real_vs_teorica.png` | Gráficas por etapa: referencia frente a odometría |
| `mision_rutas.png` | Visualización de rutas |
| `metricas/` | Métricas V3 calculadas desde los relojes sucesivos |

Ejemplo para localizar las figuras:

```bash
ls -lh scripts/resultados/milp_v3_real_nuevo_001/*.png
```

Si necesitas abrirlas **fuera de Docker**, utiliza la carpeta compartida de Ubuntu correspondiente a `~/swarmbots/swarm_ws/scripts/resultados/`.

---

## 8. Opcional: regenerar la optimización MILP desde cero

**Esta sección NO es necesaria para repetir una misión si ya conservas `milp_library` y el paquete V3.** Las búsquedas completas con CBS + Hybrid V2 pueden tardar; tampoco se afirma optimalidad global sobre las 720 asignaciones.

**Docker (sin Gazebo):**

### 8.1. Evaluar candidatas y actualizar la biblioteca

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/scripts:$PWD/src/swarm_behavior:${PYTHONPATH:-}"

python3 scripts/d9923_milp_bezier_seed100.py \
  --scenario scripts/d996_reference/scenario.json \
  --max-speed 2.0 \
  --max-accel 2.0 \
  --max-scale 2.0 \
  --transition-tau 2.0 \
  --pickup-s 23.98 \
  --delivery-s 15.98 \
  --candidates 8 \
  --include-baseline \
  --iterations 2 \
  --library-dir scripts/milp_library \
  --output-dir scripts/resultados/milp_v3_opt_nuevo_001
```

**No usar `--replace-legacy-library` por rutina**: fue una opción de migración usada al sustituir el modelo de costos anterior. Guarda una copia de la biblioteca antes de modificarla si necesitas conservar exactamente la asignación validada.

### 8.2. Exportar curvas Bézier de la asignación elegida

```bash
python3 scripts/d9925_export_milp_bezier.py \
  --scenario scripts/d996_reference/scenario.json \
  --library-dir scripts/milp_library \
  --iterations 2 \
  --output-dir scripts/resultados/milp_v3_raw_nuevo_001
```

### 8.3. Exportar el reloj sucesivo de las cuatro etapas

```bash
python3 scripts/d9931_export_successive_bundle.py \
  --scenario scripts/d996_reference/scenario.json \
  --library-dir scripts/milp_library \
  --source scripts/resultados/milp_v3_raw_nuevo_001 \
  --output-dir scripts/resultados/milp_v3_bundle_nuevo_001
```

Este exportador verifica que los costos temporales regenerados concuerden con la biblioteca MILP. Si hay discrepancias, **se rechaza la exportación**: no fuerces su uso.

### 8.4. Preflight del nuevo paquete

```bash
python3 scripts/d9927_milp_preflight.py \
  --scenario scripts/d996_reference/scenario.json \
  --library-dir scripts/milp_library \
  --timed-dir scripts/resultados/milp_v3_bundle_nuevo_001 \
  --output scripts/resultados/milp_v3_bundle_preflight_nuevo_001.json
```

Si el estado es `MILP_V3_PREFLIGHT_VALID_OFFLINE_ONLY`, se puede seguir la **secuencia de los pasos 4–7**, sustituyendo `--milp-timed-dir scripts/resultados/d9931_seed100_v3_001` por `--milp-timed-dir scripts/resultados/milp_v3_bundle_nuevo_001` en el dry-run y en la misión física. Siempre reiniciar Gazebo entre misiones.

## 9. Problemas frecuentes

- **`Carpeta de salida ya existe`:** cambiar el nombre de `--output-dir`; el supervisor no sobrescribe resultados.
- **`MILP_PREFLIGHT_REJECT`:** comprobar `scenario.json`, `best_assignment.json`, `manifest.json`, NPZ y relojes `*_clock.json`; no mezclar artefactos de distintas optimizaciones.
- **`MILP_V3: falta --enable-successive-execution`:** añadir la bandera *solo* en la ejecución física V3, tras validación.
- **Error sobre `--auto-time-scale` o `--time-scale`:** retirar el reescalado uniforme; V3 ya contiene un reloj temporal certificado offline.
- **Errores de importación del reloj:** verificar que el seguidor V3 y `successive_clock.py` estén instalados en el workspace y recompilar `swarm_behavior`.
- **Fallos TF, odometría, robots u objetos ausentes:** detener la misión y revisar Gazebo, `/clock`, nodos y `mission.log`; no comenzar sobre un mundo parcialmente ejecutado.
- **Resultados en GitHub:** normalmente **no** subir a Git `scripts/resultados/`, `build/`, `install/`, `log/` ni backups masivos; versionar el código y este README, y decidir por separado qué evidencias pequeñas conservar.

## 10. Añadir este README a GitHub (Ubuntu, fuera de Docker)

Una vez descargado este documento:

```bash
cd ~/swarmbots
mkdir -p docs
cp ~/Descargas/README_EJECUCION_MILP_SUCESIVO_V3.md docs/
git add docs/README_EJECUCION_MILP_SUCESIVO_V3.md
git diff --cached --stat
# Cuando hayas revisado los cambios y el resto del commit:
# git commit -m "docs: agregar guia de ejecucion MILP y escalado sucesivo V3"
# git push origin feature/cbs-hybrid-refinement
```

**No ejecutes `git add .` para agregar este README**, porque el repositorio contiene múltiples respaldos y resultados de simulación sin rastrear.

---

**Referencia técnica:** `docs/README_MILP_ESCALADO_SUCESIVO_V3.md` (explicación de MILP y del escalado sucesivo). Esta guía describe únicamente **cómo ejecutar** la versión ya implementada.
