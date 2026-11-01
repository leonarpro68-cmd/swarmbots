# SwarmBots `supermarket` — preparación de CBS + Hybrid V2 + V3 + D8 por semilla

**Estado:** versión experimental v1. Reutiliza los modelos ya desarrollados; no modifica la misión validada de `seed=100` ni calcula MILP al iniciar Gazebo. Nuevas semillas requieren pruebas ROS/Gazebo supervisadas; aprobar las auditorías offline **NO certifica** agarre físico ni seguimiento real.

## Qué se automatiza

Para una semilla con escenario y asignación MILP **ya generados offline** por `scripts/supermarket_milp/run_offline_seed.py`:

1. Lee y verifica la semilla y los hashes del escenario, asignación, grafo y obstáculos. La solución MILP debe tener estado `OPTIMAL_ASTAR_PROXY_OFFLINE_NOT_EXECUTABLE` y verificación exhaustiva positiva.
2. Planifica CBS dinámico **multigoal**, con obstáculos de basuritas pendientes, para `pickup1`, `delivery1`, `pickup2`, `delivery2`; exporta rutas discretas sin conflictos de CBS.
3. Comprueba los corredores geométricos para esas cuatro etapas.
4. Calcula, **etapa por etapa**, Hybrid V2 (SLSQP), cinco refinamientos y certificado continuo; después aplica V3 sucesivo con velocidades/aceleraciones máximas de 2 m/s y 2 m/s². Cada etapa guarda controles NPZ, reloj V3 y reporte. Si un paso se rechaza, se detiene sin darlo por válido.
5. Verifica independientemente los NPZ y relojes. Ejecuta el preflight **estático** de D8 con distancias nominales, 12 contratos robot/etapa y cribado geométrico local; exporta los cuatro archivos `assignment_<etapa>_MILP.json` y un `scenario_for_original_D8_seed<N>_MILP.json` si todo pasa.

El código de CBS, optimización, V3 y preflight procede de los módulos `supermarket_milp_hybrid_v2_v3_fase4_v2` y `supermarket_milp_fase5_preflight_v1`. **Los únicos cambios funcionales en estos adaptadores eliminan los bloqueos por semilla 100 y hacen los nombres de salida dependientes de la semilla.** Se conservan las condiciones de rechazo, límites dinámicos y geométricos; no se reutilizan curvas NPZ antiguas.

## Instalación — Ubuntu anfitrión, FUERA de Docker

1. Descarga y descomprime el ZIP `supermarket_seed_pipeline_v1.zip` en `~/Descargas`.
2. Ejecuta:

```bash
bash ~/Descargas/supermarket_seed_pipeline_v1/install_on_host.sh ~/swarmbots/swarm_ws
```

El instalador requiere el orquestador previo con las opciones `--authorize-noninteractive`, `--physical-policy` y `--unattended`. **No modifica** `scripts/supermarket_auto_experiment/run_auto_experiment.py` ni el launch `supermarket_full_experiment.launch.py`. Crea una copia adaptada para semillas nuevas dentro de `scripts/supermarket_seed_pipeline/` y un **segundo launch** `supermarket_full_prepared.launch.py`. Si el destino ya existe, rechaza sobrescribirlo.

Dentro de Docker (solo para registrar el launch nuevo):

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select swarm_worlds --symlink-install
source install/setup.bash
```

### Dependencias offline

Python con `numpy`, `scipy`, `shapely` y paquetes usados por el generador existente. Los scripts fuerzan un hilo para OpenBLAS/OMP/MKL en los subprocesos que lanzan. No requieren ROS 2 ni Gazebo para calcular las trayectorias.

## Uso — semilla 42

Dado que **ya generaste MILP para la semilla 42**, dentro de Docker basta:

```bash
cd ~/swarm_ws
python3 scripts/supermarket_seed_pipeline/run_full_offline.py --seed 42
```

Este comando **no vuelve a optimizar MILP**. Si falta la asignación, primero:

```bash
python3 scripts/supermarket_milp/run_offline_seed.py --seed 42
```

Para avanzar solo hasta CBS (sin esperar SLSQP):

```bash
python3 scripts/supermarket_seed_pipeline/run_full_offline.py --seed 42 --until cbs
```

Para llegar hasta V3 sin ejecutar la preparación D8:

```bash
python3 scripts/supermarket_seed_pipeline/run_full_offline.py --seed 42 --until v3
```

**No hay un plazo fijo de cálculo.** Hybrid V2 puede tardar considerablemente más para algunas etapas o semillas. No se inicia Gazebo durante el proceso.

### Archivos

```text
scripts/supermarket_seed_pipeline/
  plans/
    seed_42/
      inputs_manifest.json       # SHA y parámetros de preparación
      cbs_multigoal.json         # CBS discreto con metas multigoal
      corridors_audit.json       # cuatro corredores verificados
      phase4/
        pickup1/
          pickup1_MILP_TIMED_OFFLINE.npz
          pickup1_MILP_V3_clock.json
          pickup1_report.json
        delivery1/
          ...
        pickup2/
          ...
        delivery2/
          ...
      prepared/
        phase5_static_preflight.json
        mission_navigation_contract.json
        scenario_for_original_D8_seed42_MILP.json
        assignment_pickup1_MILP.json
        assignment_delivery1_MILP.json
        assignment_pickup2_MILP.json
        assignment_delivery2_MILP.json
```

### Ejecuciones interrumpidas y reinicio seguro

La ejecución es **reanudable**: si un CBS o una etapa ya tiene un reporte `PASS` correcto, se reutiliza sin resolverlo de nuevo. No se sobrescriben resultados. La identidad del problema queda fijada por el manifiesto con hashes del escenario, MILP, geometría y parámetros de planificación. Si alguno cambia, se bloquea: iniciar un directorio de planes completamente nuevo, conservando el anterior para auditoría.

Si el optimizador fue interrumpido a mitad de una etapa y dejó su carpeta **sin reporte completo** o con un reporte `REJECTED`, el script bloqueará la continuación. Primero conserva esa carpeta fuera del nombre de etapa, por ejemplo:

```bash
cd ~/swarm_ws
P=scripts/supermarket_seed_pipeline/plans/seed_42/phase4
mv "$P/delivery1" "$P/delivery1_failed_$(date +%Y%m%d_%H%M%S)"
python3 scripts/supermarket_seed_pipeline/run_full_offline.py --seed 42
```

**No** hagas esto para evitar un fallo geométrico: si `delivery1` fue rechazada, debes revisar su reporte, las metas y condiciones numéricas; no supone que una nueva optimización vaya a ser factible.

## Nuevo launch para una semilla con preparación COMPLETA

**Solo después de obtener** `OFFLINE_FULL_STATIC_PASS_NOT_GAZEBO_CERTIFIED` en la preparación y revisar las doce maniobras de aproximación. Inicia una sesión nueva, sin Gazebo ni soporte preexistente, y **supervisa visualmente cada robot**. Dentro de Docker:

```bash
ros2 launch swarm_worlds supermarket_full_prepared.launch.py \
  seed:=42 \
  supervised:=true
```

No es el mismo launch que ya funciona para `seed=100`. Este **nuevo** launch primero verifica hashes de escenario, CBS, certificado de cada etapa, preflight D8 y rutas absolutas de los NPZ. Si falta algo o cambió, **no inicia Gazebo**. Si pasa, carga el escenario de la semilla 42, soporte ROS y ejecuta el preflight vivo de ROS/odometría antes de mandar órdenes. El registro de ejecución tendrá una carpeta única `milp_seed42_<UTC>_<id>` y no sobrescribe experimentos.

> `supervised:=true` **no es una certificación física**. El modo sin confirmaciones intermedias requiere que alguien esté presente y pueda detenerlo de inmediato. La primera misión para cada semilla es experimental y debe ser inspeccionada especialmente en D8 (cámara, alineamiento, agarre, depósito, colisiones y desviación del seguimiento). Si falla la distancia inicial de un D8 o cualquier preflight, no modificar límites para forzarlo.

**Nunca** usar el nuevo launch con seed100 ni cambiar el original a mano:

```bash
# Sin cambios: versión validada
ros2 launch swarm_worlds supermarket_full_experiment.launch.py assignment_mode:=milp seed:=100
```

## Qué se comprobó al preparar este paquete

- Semilla 42, MILP real suministrado por el generador: CBS multigoal terminó para las cuatro etapas con **85 ticks** de suma de makespans discretos.
- Auditoría geométrica de corredores: **PASS** para las cuatro etapas.
- Hybrid V2 + cinco refinamientos + V3: **PASS** para `pickup1` de la semilla 42.
- Cribado local geométrico *orientativo* de 12 acercamientos de la semilla 42: todos dentro del rango nominal D8 y sin intersección detectada por el modelo de segmento ideal. **No reemplaza el preflight formal fase5 sobre NPZ**.
- `prepare_phase5.py` generalizado con contratos de referencia de la semilla 100: **12/12 comprobaciones estáticas PASS** (prueba de regresión, sin tocar su instalación).
- Instalador probado sobre un workspace de prueba: conserva SHA-256 del automatizador original y genera una copia sintácticamente válida con rutas D8 dependientes de semilla.

**NO SE HA VALIDADO** la fase Hybrid V2/V3 para `delivery1`, `pickup2` ni `delivery2` de la semilla 42: el proceso excedió el tiempo disponible de prueba. **NO SE HA PROBADO** este launch nuevo dentro de Gazebo con una semilla distinta de 100. Por tanto, no debe describirse la semilla 42 como una misión física funcional hasta realizar dichas pruebas.

## Fallos esperados que deben detener la misión

- `BLOCKED` por hashes distintos: mezcla de geometría, escenario, asignación o plan.
- `BLOCKED_CORRIDOR_GEOMETRY`: separación geométrica insuficiente para Bézier.
- `REJECTED_INITIAL_SLSQP_OR_CERTIFICATE`, `REJECTED_INCOMPLETE_REFINEMENT` o `REJECTED_CONTINUOUS_CERTIFICATE`: optimizador o trayectoria no certificados.
- `REJECTED_STATIC_PRECHECK`: alcance nominal D8, colisión ideal local o NPZ/clock incongruentes.
- Preflight ROS denegado: escena activa distinta, TF, odometría, estado inicial o publicadores ROS inesperados.

Nunca desbloquear una etapa cambiando solo el nombre de un archivo ni reutilizando NPZ/V3 de otra semilla.
