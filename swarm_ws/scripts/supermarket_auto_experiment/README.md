# SwarmBots — ejecución y medición automática (MILP, v1)

Orquesta **una sesión MILP de 4 etapas**, navegación CBS/Bézier/V3 y las 12 maniobras D8: tres agarres en pickup1, tres descargas en delivery1, tres agarres en pickup2 y tres descargas en delivery2. Reutiliza los ejecutores originales y los scripts de evaluación de fase 6. **No inicia Gazebo, no crea ROS nodes adicionales de movimiento, no hace reintentos.**

## Condiciones

1. Instalar fase 5 y fase 6; arrancar Gazebo `sim_summit_supermarket.launch.py` desde semilla 100, y `supermarket_support.launch.py` (solo una instancia).
2. Iniciar desde posiciones iniciales de semilla 100 y con las seis cajas disponibles. No reutilizar directorios de ensayo.
3. El NPZ de cada etapa debe coincidir con el SHA del `phase5_static_preflight.json`; el script verifica esto ANTES de emitir movimientos.
4. Hay dos modos: `--mode plan` solo genera el plan y un registro; `--mode execute` opera robots. Predeterminado es **supervisado**: al terminar las tres maniobras D8 de cada etapa exige que el usuario escriba `VERIFICADO` tras inspeccionar Gazebo. `--unattended` elimina estas pausas, pero **no verifica desprendimiento, agarre real, ni colisiones físicamente**; ACK ROS no es prueba de entrega.
5. El proceso detiene la secuencia frente a un fallo, y **no realiza recuperación ni rollback automáticos**. Si termina con error, no relanzar una misión completa sobre ese mismo Gazebo: inspeccionar estado y arrancar sesión nueva para repetir ensayo.

## Instalación HOST

```bash
bash ~/Descargas/supermarket_auto_experiment_v1/install_on_host.sh ~/swarmbots/swarm_ws
```

## Uso DOCKER (dentro de `~/swarm_ws`)

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT="$HOME/swarm_ws/scripts/supermarket_milp_phase5/results/milp_seed100_01"
RUN="$HOME/swarm_ws/scripts/resultados/experiments/milp_seed100_trial_auto01"

# Sólo plan: el directorio de plan debe ser diferente al de ejecución
python3 scripts/supermarket_auto_experiment/run_auto_experiment.py \
  --prepared "$OUT" \
  --run-dir "$HOME/swarm_ws/scripts/resultados/experiments/milp_plan_preview01" \
  --mode plan

# Ejecución SUPERVISADA (se detiene en cada límite físico para inspección)
python3 scripts/supermarket_auto_experiment/run_auto_experiment.py \
  --prepared "$OUT" --run-dir "$RUN" --mode execute \
  --confirm-new-session --confirm-supervision
```

Para ejecución SIN pausas humanas, añadir `--unattended` al comando de ejecución. Recomendación: realizar primero una misión supervisada completa desde Gazebo nuevo antes de ensayar modo sin pausas; el modo sin pausas **no certifica 6/6 entregas físicas**.

El script comprueba grafo de soporte, inicia recorder al principio, marca eventos automáticamente en `/clock`, realiza las cuatro alineaciones, dry-runs de coordinador, navegación, dry-run D8 y maniobras físicas D8, detiene el recorder al final, y genera `summary.json`. Produce archivos `automation_report.json`, `odom.csv`, `events.csv`, logs por paso. El tiempo de misión **incluye** pausas de inspección en modo supervisado; el análisis lo distingue de duración de navegación. El script **no controla** Gazebo si hay colisión o material retenido, así que la supervisión sigue siendo necesaria.

## Ensayo de referencia fijo

Esta primera versión automatiza la misión **MILP** y su asignación D8 exportada por fase 5. **No** es correcto pasarle los contratos y las basuritas de la misión fija; el automatizador fijo requiere su propio manifiesto y verificación de reportes. Para una comparación científica, la misión fija también debe grabarse con la fase 6 y con marcas equivalentes.
