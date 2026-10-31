# SwarmBots — MILP y escalado dinámico colectivo sucesivo (D9.9.29–D9.9.32)

> Estado: implementación integrada en ROS 2/Gazebo y misión completa **validada en la simulación de semilla 100**. Este documento describe la configuración validada y sus límites; **no afirma optimalidad global MILP ni garantiza el mismo comportamiento en otros escenarios**.

## 1. Resumen

SwarmBots utiliza **tres Summit**, **seis basuras** (dos verdes, dos naranjas y dos azules) y **tres depósitos** correspondientes. Cada Summit recoge dos objetos en dos rondas, con cuatro etapas de navegación coordinada:

1. `pickup1`: navegación simultánea hacia la primera basura de cada robot; luego, recogidas físicas en paralelo.
2. `delivery1`: navegación simultánea hacia los depósitos; luego, una o más oleadas físicas, evitando dos entregas simultáneas al mismo depósito.
3. `pickup2`: navegación simultánea hacia las tres basuras restantes; luego, recogidas físicas.
4. `delivery2`: navegación simultánea y entregas físicas por oleadas.

La cadena de navegación conserva **A* + CBS + Hybrid V2/Bézier quínticas**. La nueva capa MILP elige **qué objetos** atiende cada robot y en qué orden, mientras que el **escalado temporal colectivo sucesivo** reduce el tiempo de ejecución de las curvas planificadas sin cambiar sus puntos de control espaciales. El seguidor **V3** reproduce el reloj sucesivo en Gazebo.

## 2. MILP: qué optimiza y qué no

Para cada asignación candidata, el evaluador offline:

1. Determina objetivos de `pickup1`, `delivery1`, `pickup2` y `delivery2`, respetando que cada basura se recoja exactamente una vez y que vaya al depósito de su color.
2. Planifica rutas mediante A* y coordinación de conflictos mediante CBS.
3. Obtiene trayectorias suaves quínticas mediante el planificador Hybrid V2.
4. Calcula un **reloj colectivo sucesivo** para las tres trayectorias de cada etapa y toma su duración común final.
5. Suma las cuatro duraciones y una estimación del tiempo de maniobras físicas. Compara candidatas certificadas y conserva la mejor conocida por semilla y modelo temporal.

La función objetivo usada en las pruebas fue:

\[
C=\sum_{r=1}^{2}\left(T^{\mathrm{sucesivo}}_{\mathrm{pickup},r}+23.98\ \mathrm{s}+T^{\mathrm{sucesivo}}_{\mathrm{delivery},r}+15.98\ \mathrm{s}\cdot W_r\right),
\]

donde `W_r` es la cantidad de oleadas físicas necesarias en la entrega de la ronda `r`. **23,98 s** y **15,98 s** son presupuestos/modelos de costo, **no cotas superiores demostradas** de ejecución física.

**Alcance de la optimalidad:** HiGHS resuelve la selección entre las candidatas evaluadas y certificadas; la búsqueda no garantiza que se haya evaluado el conjunto completo de 720 asignaciones posibles. Por tanto, el resultado es la *mejor candidata certificada* dentro del conjunto explorado, no un óptimo global demostrado.

### Biblioteca persistente

Se guarda bajo `scripts/milp_library/seed_<seed>/` (por ejemplo `seed_100`) con información del escenario, mejor asignación e historial. Los costos antiguos, calculados sin el modelo sucesivo, **no son comparables** con los nuevos. El modelo actual se identifica como:

`common_collective_successive_v1`

La biblioteca y los artefactos exportados deben corresponder a la **misma semilla, escenario, asignación y versión de modelo temporal**. No sustituir únicamente el JSON de asignación dejando NPZ viejos.

## 3. Escalado dinámico colectivo sucesivo

### 3.1. Idea central

CBS/Hybrid produce para cada robot una trayectoria espacial `r_i(τ)` parametrizada por un **tiempo virtual** `τ`. Las tres usan el mismo reloj virtual. Se define una función de velocidad del reloj `q(τ)>0`:

\[
\frac{d\tau}{dt}=q(\tau),\qquad
 t(\tau)=\int_0^\tau\frac{du}{q(u)},\qquad
 x_i(t)=r_i(\tau(t)).
\]

`q` es **colectivo**: en cada instante virtual es el mismo para todos los Summit que aún se desplazan. En cuanto un robot completa su trayectoria, deja de restringir los incrementos posteriores del factor. El factor sube suavemente en los intervalos restantes, de forma sucesiva, según los límites dinámicos del conjunto activo. El método se calcula **offline**, no por realimentación de odometría cuando un robot termina físicamente.

### 3.2. Por qué no usar un factor fijo

Con reescalado uniforme existe un único `k` para toda la etapa y todos los robots: `t_nuevo=t_original/k`. Es conservador incluso cuando el robot que restringía el factor ya ha terminado. El escalado sucesivo puede aumentar `q(τ)` después de esa finalización, acortando el tiempo del último robot; **no garantiza mejora estricta en todas las etapas**.

### 3.3. Transiciones suaves y aceleración

No se permite un salto instantáneo en `q`, pues provocaría discontinuidad en la velocidad comandada. Las transiciones se representan mediante rampas suaves (smoothstep quíntico) de duración virtual controlada por `transition_tau`. Por regla de la cadena:

\[
\mathbf v_i(t)=q(\tau)\,\mathbf r_i'(\tau),
\]

\[
\mathbf a_i(t)=q(\tau)^2\,\mathbf r_i''(\tau)
+q(\tau)\,q'(\tau)\,\mathbf r_i'(\tau).
\]

El segundo término incluye explícitamente la aceleración asociada al **cambio del factor**; es insuficiente comprobar solo `q² r''`. En la versión validada se emplearon límites de planificación de **2,0 m/s** y **2,0 m/s²**, `max_scale=2.0` y `transition_tau=2.0` (en unidades de tiempo virtual). El motor D9.9.29 utiliza cotas conservadoras mediante propiedades de Bernstein de Bézier; una cota de 2,0 m/s² **no implica** que la aceleración puntual dibujada en una gráfica alcance exactamente 2,0.

### 3.4. Relación con CBS y la ejecución real

Como todos los robots comparten la transformación monótona `τ↔t`, se conservan las coincidencias relativas de instantes virtuales que CBS había coordinado. Esto no constituye por sí solo una nueva prueba de ausencia de colisiones físicas con errores de seguimiento: se conservan las protecciones de ejecución (barreras, watchdog, límites, validaciones y estados).

**V3 no es una lista de duraciones Bézier reescaladas independientemente**: el reloj es continuo. El seguidor evalúa la posición sobre las curvas originales en `τ(t)` y la velocidad mediante el factor actual `q(τ)`. En modo MILP V3 **no activar `--auto-time-scale`**, pues generaría una segunda transformación temporal. El modo `original` mantiene el protocolo V2 y su comportamiento anterior.

## 4. Componentes implementados

| Componente | Responsabilidad |
|---|---|
| `scripts/d9923_milp_bezier_seed100.py` | Evaluar candidatas A*/CBS/Hybrid con costo sucesivo y seleccionar solución offline |
| `scripts/d9924_assignment_library.py` | Biblioteca persistente por semilla/modelo e historial |
| `scripts/d9929_sucesivo_offline.py` | Construcción y certificación offline del reloj común sucesivo |
| `scripts/d9925_export_milp_bezier.py` | Regeneración de las curvas de la solución elegida |
| `scripts/d9931_export_successive_bundle.py` | Exportación de curvas y relojes V3 para las cuatro etapas; comprobación contra tiempos de biblioteca |
| `scripts/d9927_milp_preflight.py` | Identidad, consistencia, integridad y autorización offline de los artefactos |
| `scripts/d9913_stage_coordinator.py` | Preparación y despacho de navegación temporal V3 |
| `src/swarm_behavior/.../timed_protocol.py` y `successive_clock.py` | Protocolo/interpretación del reloj en el seguidor (ubicación exacta según paquete ROS instalado) |
| `scripts/d9917_physical_stage.py`, `d9918_delivery_waves.py`, `d9920_full_physical_mission.py` | Recogidas físicas, entregas por oleadas y ejecución de las cuatro etapas |
| `scripts/d9922_mission_with_telemetry.py`, `scripts/d9932_v3_metrics.py` | Misión integral, reporte y contraste teórico vs odometría |

D9.9.32 corrige además la carga de módulos de V3, la compatibilidad del preflight con las maniobras físicas y la interpretación de los tiempos virtuales en telemetría.

## 5. Resultados comprobados — semilla 100

### 5.1. Selección MILP con el nuevo costo

En el experimento offline se exploraron **9 candidatas certificadas** entre **720 asignaciones posibles**. El solver HiGHS finalizó correctamente y eligió:

| Summit | Primera basura | Segunda basura |
|---|---|---|
| Summit 0 | V2 verde | A1 azul |
| Summit 1 | N2 naranja | V1 verde |
| Summit 2 | A2 azul | N1 naranja |

Cada ronda requiere una sola oleada de entrega (tres colores distintos).

### 5.2. Tiempos de navegación estimados (segundos)

| Etapa | Uniforme | Sucesivo V3 | Ahorro |
|---|---:|---:|---:|
| `pickup1` | 11,9607 | 10,7745 | 1,1862 |
| `delivery1` | 31,3607 | 29,3978 | 1,9629 |
| `pickup2` | 20,0674 | 18,6596 | 1,4078 |
| `delivery2` | 25,8554 | 18,1170 | 7,7384 |
| **TOTAL** | **89,2441** | **76,9488** | **12,2953** |

Costos de la mejor candidata: **76,9488 s** de navegación estimada + **79,92 s** de operaciones físicas presupuestadas = **156,8688 s** de misión estimada. Estos tiempos **no son mediciones del tiempo total real transcurrido en Gazebo**.

Para referencia, la asignación histórica evaluada con el **mismo** modelo sucesivo obtuvo **212,6502 s** de costo estimado (100,7702 s navegación + 111,88 s operaciones físicas). Esa comparación corresponde al conjunto y al modelo evaluados, no a un ensayo estadístico con múltiples semillas.

### 5.3. Primera ejecución física V3 en Gazebo

Se completó con éxito la misión de la semilla 100. El reporte final declara:

- `status = DONE_MISSION_WITH_TELEMETRY_AND_METRICS`
- `mission_status = DONE_ALL_FOUR_PHYSICAL_STAGES_ACK`
- `mission_exit_code = 0`, `problems = []`
- Cuatro etapas registradas con `time_model = successive`, sin `auto_time_scale` adicional.

El informe de `pickup1_navigation.json` registró duraciones planificadas por robot de **6,8347 s (Summit 0)**, **10,7745 s (Summit 1)** y **9,5652 s (Summit 2)**. El último robot, Summit 1, determina el tiempo de esa etapa. Se generó gráfica de teoría frente a odometría en las cuatro etapas. El estado de éxito **no implica coincidencia perfecta entre aceleraciones certificadas, teóricas graficadas y odometría**; esa diferencia se dejó fuera del alcance de esta versión.

## 6. Flujo de reproducción — ejemplo semilla 100

Requiere un workspace ROS 2 Jazzy con D9.9.29–D9.9.32 instalados, `swarm_behavior` compilado y el escenario correspondiente. Estos comandos sirven como ejemplo y deben ajustarse a la ruta local real de los resultados:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Evaluación de candidatas y renovación de biblioteca (offline)
python3 scripts/d9923_milp_bezier_seed100.py \
  --scenario scripts/resultados/d8/seed_100_run_007/scenario.json \
  --max-speed 2.0 --max-accel 2.0 \
  --max-scale 2.0 --transition-tau 2.0 \
  --pickup-s 23.98 --delivery-s 15.98 \
  --candidates 8 --include-baseline --iterations 2 \
  --library-dir scripts/milp_library \
  --replace-legacy-library \
  --output-dir scripts/resultados/d9930_seed100_200_001

# Exportación de la asignación elegida y del reloj V3
python3 scripts/d9925_export_milp_bezier.py \
  --scenario scripts/d996_reference/scenario.json \
  --library-dir scripts/milp_library --iterations 2 \
  --output-dir scripts/resultados/d9931_seed100_raw_001
python3 scripts/d9931_export_successive_bundle.py \
  --scenario scripts/d996_reference/scenario.json \
  --library-dir scripts/milp_library \
  --source scripts/resultados/d9931_seed100_raw_001 \
  --output-dir scripts/resultados/d9931_seed100_v3_001

# Verificación sin movimiento
python3 scripts/d9927_milp_preflight.py \
  --scenario scripts/d996_reference/scenario.json \
  --library-dir scripts/milp_library \
  --timed-dir scripts/resultados/d9931_seed100_v3_001 \
  --output scripts/resultados/d9932_seed100_preflight.json

# Dry-run sin --execute
python3 scripts/d9922_mission_with_telemetry.py \
  --assignment-method milp \
  --milp-library-dir scripts/milp_library \
  --milp-timed-dir scripts/resultados/d9931_seed100_v3_001 \
  --reference-dir scripts/d996_reference \
  --nav-speed-limit 2.0 --nav-accel-limit 2.0 \
  --output-dir scripts/resultados/d9932_seed100_dry_001
```

Una vez comprobados `MILP_V3_PREFLIGHT_VALID_OFFLINE_ONLY`, `DRY_RUN_DONE`, la compilación V3 y el reinicio de Gazebo en mundo limpio, la ejecución supervisada añade `--execute --enable-successive-execution --confirm-runtime-v2 --confirm-cargo-inspected`, con un nuevo directorio de salida. **Nunca** ejecutar V3 con `--auto-time-scale`.

## 7. Limitaciones y mejoras futuras

- La búsqueda MILP no ha certificado optimalidad global sobre las 720 asignaciones.
- Las duraciones físicas en la función objetivo son **estimaciones**, no tiempos observados fijos.
- La certificación Bernstein de velocidad/aceleración es **conservadora**; no debe confundirse con máximos puntuales observados.
- La validación física presentada corresponde a una ejecución exitosa con la semilla 100; repetir con otras semillas y condiciones antes de generalizar.
- Evitar subir automáticamente archivos de ejecución voluminosos (`scripts/resultados/`, registros ROS, `build/`, `install/`, `log/`) sin decidir explícitamente qué evidencias conservar en el repositorio.

## 8. Versiones relacionadas

- **D9.9.29:** motor offline del reloj sucesivo.
- **D9.9.30:** costo MILP y renovación de biblioteca.
- **D9.9.31:** protocolo/ejecutor temporal V3 y exportación.
- **D9.9.32:** ajustes de integración, entregas y telemetría.

