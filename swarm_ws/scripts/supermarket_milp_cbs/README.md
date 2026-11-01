# Fase 3: MILP → CBS discreto para supermercado, semilla 100

**Solo offline.** No sustituye Bézier Hybrid V2, los relojes V3, la validación de corredores ni las maniobras D8. No utilizar este JSON como ruta ejecutable de los robots.

Esta adaptación utiliza una **copia intacta** de `supermarket_planning/cbs.py` y `geometry.py` contenida en `supermarket_astar_cbs_v1.zip`, sin tocar las instalaciones ROS existentes. El grafo procede del paquete de las fases 1 y 2. Usa como entrada `assignment_milp_reference.json` (MILP con un color por ronda) y `seed_100_reference.json`.

## Ejecución en Ubuntu (FUERA de Docker)

Extraer `supermarket_milp_cbs_fase3_v1.zip` en Descargas y ejecutar:

```bash
bash "$HOME/Descargas/supermarket_milp_cbs_fase3_v1/install_on_host.sh" "$HOME/swarmbots/swarm_ws"
```

El instalador **no sobrescribe** un directorio preexistente. Instala solo en `scripts/supermarket_milp_cbs` y no necesita recompilar ROS 2.

## Ejecución en Docker

```bash
cd ~/swarm_ws/scripts/supermarket_milp_cbs
python3 test_phase3.py
# El paquete contiene un reporte de referencia; elige una salida NUEVA al ejecutar
python3 plan_milp_cbs.py \
  --assignment ~/swarm_ws/scripts/supermarket_milp/results/seed_100/assignment_milp_one_color_per_round.json \
  --scenario ~/swarm_ws/config/supermarket/seed_100.json \
  --output ~/swarm_ws/scripts/supermarket_milp_cbs/results/milp_cbs_live_seed100.json
```

**Importante:** si ya existe `milp_cbs_live_seed100.json`, el programa se detiene para evitar sobrescribirlo. No uses la salida de una semilla distinta. Comprueba manualmente que el escenario activo coincide con `seed_100_reference.json` antes de confiar en el reporte.

## Modelo geométrico y restricciones

- `pickup1`: inicios reales de cada robot, metas discretas `approach_grid` de la basurita asignada por MILP.
- `delivery1`: inicia al terminar `pickup1`, va al nodo de aproximación del depósito de cada color.
- `pickup2`: inicia al terminar `delivery1`, va a `approach_grid` de la segunda basurita.
- `delivery2`: inicia al terminar `pickup2`, va al depósito correspondiente.
- Las paradas discretas para depósitos están **fijadas** a azul `(16,12)` = `(5,-10)`, verde `(10,19)` = `(-2.5,-1.25)` y naranja `(8,32)` = `(-5,15)`; son puntos *proxy*, no necesariamente los puntos de aproximación ideales de las nuevas asignaciones o de los procedimientos D8. Esto debe adaptarse/validarse antes de Bézier.
- CBS evita conflictos por distancia entre centros de robots mediante su regla geométrica del paquete original. No certifica una trayectoria continua con tracking en Gazebo, no valida maniobras con cargas, ni colisiones con cajas depositadas.
- Esperas y desplazamientos cardinales/diagonales cuentan un tick por transición: **63 ticks no son 63 segundos**.
- El estado `OFFLINE_CBS_ONLY_NOT_EXECUTABLE` y `execution_authorized:false` son intencionales.

## Reporte de referencia

- `pickup1`: 10 ticks de makespan, 1 nodo CT expandido.
- `delivery1`: 18 ticks, 4 nodos CT.
- `pickup2`: 11 ticks, 1 nodo CT.
- `delivery2`: 24 ticks, 1 nodo CT.
- Suma de makespans discreta: **63 ticks**.

**Siguiente fase:** generar y certificar Bézier Hybrid V2 + sincronización V3 para estos **nuevos** CBS; no reutilizar los NPZ ni `clock.json` de la misión fija anterior. Además, revisar la compatibilidad D8 de cada meta final.
