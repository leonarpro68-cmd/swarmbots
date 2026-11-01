# SwarmBots — Fase 4 MILP → CBS multigoal → Hybrid V2 → reloj V3

**Ejecución exclusivamente offline. No contiene autorización para mover robots.**

## Estado y alcance verificado

Entrada: asignación MILP semilla 100 y planificación CBS *corregida* por multigoal, 84 ticks acumulados. El reporte de la fase 3 original (63 ticks) **NO** cumple el contrato geométrico y no debe utilizarse con este paquete.

Sobre el plan MILP corregido que se adjunta como referencia se verificó:

| Etapa | Tick CBS | SLSQP original | Refinamientos aceptados | Certificado continuo | V3 sucesivo (s) |
|---|---:|---|---:|---|---:|
| `pickup1` | 9 | PASS | 5/5 | PASS | 11.7972 |
| `delivery1` | 20 | PASS | 5/5 | PASS | 19.9450 |
| `pickup2` | 30 | PASS | 5/5 | PASS | 31.4856 |
| `delivery2` | 25 | PASS | 5/5 | PASS | 22.8889 |
| **Total** | **84** | | **20/20** | **4/4** | **86.1166** |

Los tiempos V3 corresponden a un perfil temporal conservador elegido offline con los límites **2.0 m/s** y **2.0 m/s²**. No son los tiempos observados en Gazebo. La optimización MILP de fases 1–2 minimizó otro modelo proxy (A*) y por sí misma no predice estas duraciones V3. Por tanto, todavía **no se ha demostrado** mejora de tiempo respecto de la misión fija anterior.

Los certificados verifican distancias sobre *envolventes convexas de controles Bézier*, C², velocidades/aceleraciones por Bernstein y relojes temporales colectivos. Los límites geométricos conservados son distancia centro–estantería o pared 0.60 m (Summit R=0.55 m + 0.05 m libre), distancia entre centros de robots 1.15 m y zona de exclusión de cajas de **0.09 × 0.09 m** (radio 0.66364 m), además de política de depósitos 0.50 m. El margen adicional `0` del certificado es **además** de los 5 cm, no una nueva reserva de seguimiento.

**Pendiente obligatorio:** verificar alcance y orientación del gripper D8 desde cada meta CBS. En esta asignación algunas posiciones de aproximación están a 1.7–2.1 m de sus basuritas. La comprobación de distancia no demuestra que D8 pueda alcanzarlas, sujetarlas o descargarlas. Ningún resultado de este paquete permite aún ejecutar la misión MILP en Gazebo.

## Qué contiene

- `run_phase4.py`: orquestador conservador, con entrada CBS de la fase anterior. No modifica los scripts de las fases anteriores.
- `original_hybrid/slsqp.py`, `original_hybrid/geometry_corridors.py`, etc.: solver SLSQP **original** de SwarmBots; no se sustituye su formulación de continuidad C².
- `corridors_supermarket.py`, `refinement_supermarket.py`, `safety_supermarket.py`: adaptación del supermercado con rectángulos, exclusiones dinámicas, cinco refinamientos por 32 muestras y certificados de convex hull.
- `original_v3/d9929_sucesivo_offline.py`, `successive_clock.py`: reloj V3 original; todos los Summit comparten la misma deformación de tiempo.
- `verify_offline.py`: relee controles, relojes y reportes; recalcula independientemente límites dinámicos por robot.
- `audit_d8_proximity.py`: diagnostica la distancia de aproximación, pero **no marca** objetivos como alcanzables.
- `reference_cbs_seed100.json`, `reference_outputs/`: resultados ejemplares calculados y contrastables; **no** son planes listos para ROS 2.
- `tests/test_offline.py`: 6 pruebas de integridad y seguridad.

## Instalar en Ubuntu (FUERA de Docker)

Extraer `supermarket_milp_hybrid_v2_v3_fase4_v2.zip` en `~/Descargas`, dejando la carpeta del mismo nombre. Ejecutar:

```bash
bash ~/Descargas/supermarket_milp_hybrid_v2_v3_fase4_v2/install_on_host.sh \
  ~/swarmbots/swarm_ws
```

Se crea **únicamente** `~/swarmbots/swarm_ws/scripts/supermarket_milp_hybrid_v2_v3/` (si ya existe se rechaza para no sobrescribir). No se modifica `sim_summit.launch.py`, `supermarket_support.launch.py`, CBS anterior ni las trayectorias que se ejecutaron exitosamente.

## Ejecutar dentro de Docker, SIN Gazebo

Requiere Python con `numpy`, `scipy` y `shapely`. Limitar hilos BLAS para evitar latencia e inestabilidad numérica:

```bash
cd ~/swarm_ws/scripts/supermarket_milp_hybrid_v2_v3
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 -m unittest discover -s tests -v
```

Ahora utilizar el **CBS corregido de tu propia sesión**, no el CBS original de la fase 3:

```bash
CBS=/tmp/milp_cbs_multigoal_corr_live_v1.json
SC=~/swarm_ws/config/supermarket/seed_100.json
ls -l "$CBS" "$SC"
```

Si falta el archivo CBS, puedes volver a generarlo desde `scripts/supermarket_milp_corridor_fix` usando la asignación real de fases 1–2 y un nuevo nombre de salida. No reutilices un reporte de otra semilla o escenario.

### Paso 1: ejecutar la fase 4, una etapa a la vez

Las cuatro ejecuciones son independientes y escriben resultados en carpetas separadas, de modo que no se sobrescriba un éxito anterior:

```bash
for STAGE in pickup1 delivery1 pickup2 delivery2; do
  echo "========== $STAGE =========="
  python3 run_phase4.py \
    --cbs "$CBS" \
    --scenario "$SC" \
    --stage "$STAGE" \
    --maxiter 450 \
    --refinements 5 \
    --max-speed 2.0 \
    --max-accel 2.0 \
    --output-dir "results/milp_100_phase4_v1/$STAGE" || break
done
```

Cada etapa debe imprimir `PASS_OFFLINE_HYBRID_V2_V3_NOT_EXECUTABLE`. Si falla, **no repitas cambiando solo el nombre del reporte**. Inspecciona `<stage>_report.json` y corrige el origen del error. El script no deja un NPZ temporal si rechaza la etapa.

### Paso 2: comprobar las cuatro salidas

```bash
python3 verify_offline.py --results results/milp_100_phase4_v1
```

Esperado: `PASS_OFFLINE_ARTIFACTS_NOT_EXECUTABLE` y cuatro `PASS`.

### Paso 3: diagnóstico D8 (sin movimiento)

```bash
python3 audit_d8_proximity.py \
  --cbs "$CBS" --scenario "$SC" \
  --output results/milp_100_phase4_v1/d8_distance_only.json
```

**No** debemos conectar los nuevos `*.npz` directamente al ejecutor `d9913_stage_coordinator.py`: primero hay que construir/validar el contrato del ejecutor ROS y los puntos efectivos de aproximación/manipulación D8, y luego ejecutar los preflight. Los NPZ del paquete contienen `R{i}_controls` y `R{i}_durations_s`, más el reloj V3 separado, pero **solo son artefactos offline**.

## Resultados de referencia en el paquete

Los tiempos, certificados y comprobaciones citados se basan en `reference_outputs/` calculados con `reference_cbs_seed100.json`. Esas referencias no reemplazan los resultados obtenidos del CBS y escenario actuales en tu Docker; hay que regenerar y comparar hashes.

### Nota numérica

Algunas separaciones mínimas del certificado se encuentran próximas a cero por redondeo de punto flotante, respecto al radio inflado y a los **5 cm libres ya incluidos**. No se afirma una reserva adicional de 5 cm frente al error de seguimiento. Las partes móviles y los errores físicos del gripper quedan para la fase de ejecución supervisada.
