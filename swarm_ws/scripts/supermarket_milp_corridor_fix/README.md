# Corrección geométrica MILP → CBS → Hybrid V2 (offline)

## Por qué existe

El primer plan MILP→CBS (fase 3) generaba metas fijas de entrega (`DEP_APPROACH`) y no aplicaba obstáculos circulares dinámicos de las basuritas a su búsqueda. Por eso el reporte de fase 4 `BLOCKED_CORRIDOR_GEOMETRY` era coherente: CBS discreto no implicaba corredores seguros para Hybrid V2.

Esta variante **no modifica la asignación MILP**. Adapta los módulos **originales** `DynamicTrashGraph`, `approach_vertices`, `deposit_goals` y `solve_multi`, del planificador multigoal ya utilizado en SwarmBots. Cada etapa usa como obstáculos todas las basuritas pendientes, y los depósitos tienen zona virtual de exclusión 0.50 m (incluso durante entrega, cuyo centro no debe atravesarse). La exclusión de colisión para las cajas se actualiza de 12 cm a **9 cm**; el grafo sigue usando su margen conservador propio de 0.936 m. Las metas de la fase 3 se sustituyen por vecinos transitables elegidos por CBS. No se relajan los 0.60 m de estanterías/paredes ni los 1.15 m interrobot.

## Reproducción verificada offline en este paquete

- `pickup1`: PASS
- `delivery1`: PASS
- `pickup2`: PASS
- `delivery2`: PASS
- Makespan discreto acumulado: **84 ticks** (no segundos, ni duración V3). El plan previo reportaba 63 ticks, pero no cumplía las exclusiones de esta validación, por lo que no es una comparación de eficiencia equivalente.
- **PASS_CORRIDOR_PREFLIGHT_ONLY**: solo certifica que los tramos discretos caben en corredores factibles (antes de SLSQP); NO autoriza ejecución física.

Los extremos elegidos están a ~**1.7–2.1 m** del centro de varias basuritas, distancias que podrían exceder el radio válido de maniobra local D8 según orientación. La comprobación de alcance físico es **obligatoria** antes de Gazebo. Tampoco están ejecutados SLSQP, cinco refinamientos, certificación de controles ni reloj colectivo V3. La solución MILP es óptima para su proxy A*, no para el tiempo real de misión.

## Instalación — Ubuntu host

Extrae ZIP en `~/Descargas/supermarket_milp_corridors_corregidos_v1`, luego:

```bash
bash ~/Descargas/supermarket_milp_corridors_corregidos_v1/install_on_host.sh ~/swarmbots/swarm_ws
```

Quedará en `~/swarmbots/swarm_ws/scripts/supermarket_milp_corridor_fix/`. No pisa la fase 3 anterior ni los `launch`.

## Dentro de Docker

No hace falta arrancar Gazebo ni compilar colcon. Se requiere `numpy` y `shapely`, instalados para fase 4.

```bash
cd ~/swarm_ws/scripts/supermarket_milp_corridor_fix
OPENBLAS_NUM_THREADS=1 python3 test_fix.py

# Opción A: con los datos de referencia del mismo paquete:
OPENBLAS_NUM_THREADS=1 python3 plan_milp_multigoal.py \
  --output /tmp/milp_cbs_multigoal_corr_v1.json

# Opción B: con los resultados reales de fases 1-2 del workspace:
# Verifica antes la ruta exacta del JSON de asignación.
OPENBLAS_NUM_THREADS=1 python3 plan_milp_multigoal.py \
  --assignment ~/swarm_ws/scripts/supermarket_milp/results/seed_100/assignment_milp_one_color_per_round.json \
  --scenario ~/swarm_ws/config/supermarket/seed_100.json \
  --output /tmp/milp_cbs_multigoal_corr_live_v1.json

OPENBLAS_NUM_THREADS=1 python3 audit_phase4.py \
  --cbs /tmp/milp_cbs_multigoal_corr_live_v1.json \
  --scenario ~/swarm_ws/config/supermarket/seed_100.json \
  --geometry supermarket_obstaculos.json \
  --output /tmp/milp_phase4_corr_live_v1.json
```

El auditor verifica SHA256 de sus entradas y marca siempre `execution_authorized: false`. No se sobrescriben reportes anteriores, elige nombres nuevos si los archivos de salida existen.

**Pendiente de integración:** usar estos `paths_grid` y `goal_options` para SLSQP Hybrid V2 y V3, validar los puntos D8 con las posiciones reales y comprobar certificados continuos. Nunca reutilizar trayectorias NPZ de asignación fija.
