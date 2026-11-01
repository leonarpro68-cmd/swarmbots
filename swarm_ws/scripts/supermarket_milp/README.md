# SwarmBots — MILP Supermarket Fases 1 y 2 (OFFLINE)

**Estado:** cálculo A* + optimización MILP comprobados. **Sin comandos ROS / Gazebo.** Estos resultados **no son trayectorias ejecutables** ni validación CBS/Bézier/V3/D8.

## Fuentes de datos

- `data/seed_100.json` extraído del generador de escenarios entregado, mismo escenario de 3 robots, 6 basuritas y 3 depósitos. Se verifica contra la versión viva antes de emplearlo en nuevas simulaciones.
- `data/supermarket_grafo.json`, grafo de 588 vértices, 1786 aristas, resolución 1.25 m, exclusión preliminar de obstáculos 0.936 m, generado en el proyecto `supermarket_escenarios_v1`.
- Los puntos de aproximación a basuras corresponden al JSON de escenario; no son necesariamente los objetivos finales elegidos por CBS, que pueden diferir por 1 o más celdas.
- Para finalizar cada entrega se usan paradas fijas cercanas a depósitos, observadas en la misión validada: azul `(5,-10)`, verde `(-2.5,-1.25)`, naranja `(-5,15)`. El modelo mantiene la misma parada en ambas rondas por ahora; **no optimiza** dónde detenerse alrededor del depósito. No es equivalente a la segunda entrega real de naranja y verde.

## Qué optimiza

Variables binarias `y_{r,a,b}`: robot `r` recoge la basurita `a` en ronda 1 y `b` en ronda 2, `a ≠ b`. Con 3 robots y 6 objetos hay 90 binarias. Variables continuas `T_pickup1, T_delivery1, T_pickup2, T_delivery2` representan el mayor tiempo de viaje individual de la etapa. Minimiza `T_pickup1 + T_delivery1 + T_pickup2 + T_delivery2`, con un desempate diminuto de longitud. Costos obtenidos mediante A* en el grafo, convertidos a tiempo a velocidad proxy de 2.0 m/s (NO velocidad constante certificada del robot).

Restricciones: cada robot obtiene una pareja, cada basura se atiende una vez, y `T_stage` supera todos los tiempos individuales. Opción `--one-color-per-round`: exactamente una basurita de cada color en cada ronda para evitar acumulación intencional en depósitos; no prueba ausencia de conflictos entre robots.

Se comprueba la solución MILP de forma independiente por enumeración exhaustiva de las 720 asignaciones posibles (288 si se impone una por color/ronda). Esto certifica el **óptimo del costo proxy** dentro del universo indicado; **no** el óptimo físico global de Gazebo.

## Instalar en el host Ubuntu (fuera de Docker)

Extraer `supermarket_milp_fases1_2_v1.zip` en Descargas (queda la carpeta `supermarket_milp_fases1_2_v1`).

```bash
bash "$HOME/Descargas/supermarket_milp_fases1_2_v1/install_on_host.sh" \
  "$HOME/swarmbots/swarm_ws"
```

Instala solo `scripts/supermarket_milp`. No hace falta `colcon build` (no cambia paquetes ROS). Si ya existen los reportes de salida, el instalador **no los sobrescribe**. Antes de reejecutar pruebas usa otro nombre de salida o mueve esos reportes.

## Dentro de Docker — reconstruir A* y MILP

```bash
cd ~/swarm_ws
python3 -c 'import scipy; print("SciPy",scipy.__version__)'

python3 scripts/supermarket_milp/build_costs.py \
  --scenario scripts/supermarket_milp/data/seed_100.json \
  --graph scripts/supermarket_milp/data/supermarket_grafo.json \
  --output scripts/supermarket_milp/results/seed_100/astar_costs.json

python3 scripts/supermarket_milp/solve_assignment.py \
  --costs scripts/supermarket_milp/results/seed_100/astar_costs.json \
  --one-color-per-round \
  --output scripts/supermarket_milp/results/seed_100/assignment_milp_one_color_per_round.json

python3 -m unittest discover -s scripts/supermarket_milp/tests -v
```

Para analizar TODAS las 720 combinaciones sin limitar una basurita por color/ronda, omite `--one-color-per-round` y usa salida `assignment_milp.json`.

**Si el escenario activo difiere del JSON incluido**, ejecutar `build_costs.py --scenario ~/swarm_ws/config/supermarket/seed_100.json` con el archivo vivo y comprobar su integridad; no usar reportes antiguos como si fueran equivalentes. La configuración del grafo debe corresponder al escenario utilizado.

## Resultado de referencia, semilla 100

| Robot | Pickup1 | Pickup2 |
|---|---|---|
| summit0 | `trash_V1_verde` | `trash_N1_naranja` |
| summit1 | `trash_A1_azul` | `trash_V2_verde` |
| summit2 | `trash_N2_naranja` | `trash_A2_azul` |

Costo proxy fijo `47.829320s`; MILP `43.776019s`; diferencia `4.053301s` (~`8.47%`) bajo las simplificaciones arriba. La misma asignación respeta una basurita de cada color por ronda. Reportes en `results/seed_100/`.

## Próximo paso (NO incluido)

Recalcular objetivos discretos para la nueva asignación, planificar las **4 etapas** mediante CBS, construir corredores, certificar Bézier Hybrid V2 con seguridad/curvatura y los relojes colectivos V3, generar **nuevos** NPZ/clock verificados, y adaptar los JSON para maniobras D8. No ejecutar la nueva asignación con los NPZ de la misión fija anterior.
