# SwarmBots Hybrid V2 — Etapa C.2 (offline)

Implementación incremental sobre las etapas A, B y C. No modifica los launch de Gazebo, ROS 2 ni los controladores.

## Qué cambia

- `geometry_corridors.corridors(..., operational_margin_m=0.05)`: añade una reserva **extra** a radios de exclusión ya existentes, a paredes y a la separación interrobot. En cada semiplano entre robots se añade la mitad de la reserva total.
- `refinement.sampled_corridors(..., operational_margin_m=0.05)` y `refine(...)`: mantienen la geometría y la estrategia SLSQP + refinamiento por muestras del experimento, ahora reconstruyendo corredores con reserva operacional.
- `safety.certify(..., preserve_intermediate_waits=False)`: permite relajar WAIT intermedios sin imponer el paso por cada waypoint CBS; exige tramos estacionarios después de la llegada y verifica seguridad de todas las curvas completas mediante envolventes convexas, no solo puntos muestreados.
- `pipeline_c2.py`: ejecuta planificación y refinamiento, certifica y exporta JSON de estado. Solo exporta puntos de control `.npz` cuando la certificación se aprueba.
- `test_hybrid_v2_stage_c2.py`: pruebas de reservas, política de WAIT y rechazo explícito.

**Alcance de la certificación**: seguridad matemática conservadora para el modelo circular y sincronía temporal ideal; **no certifica** por sí sola errores reales de odometría, modelo dinámico del Summit, latencias ni seguimiento en Gazebo. Un margen de 5 cm es parámetro experimental, no garantía física. El escalado global produce un `dt_s` común para todas las piezas y robots.

## Instalar

En Ubuntu (fuera de Docker):

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_hybrid_v2_etapa_C2_completo.zip -d ~/swarmbots
```

Dentro de Docker:

```bash
docker exec -it swarm_dev bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m pytest -q src/swarm_behavior/test/test_hybrid_v2*.py
```

## Simulación completa, entrega ronda 2

```bash
python3 -m swarm_behavior.swarm_planning.hybrid_v2.pipeline_c2 \
  --operational-margin 0.05 --iterations 5 \
  --max-speed 1.20 --max-accel 1.00 \
  --output ~/swarm_ws/scripts/resultados/hybrid_v2_C2_entrega_ronda2.json \
  --controls-output ~/swarm_ws/scripts/resultados/hybrid_v2_C2_entrega_ronda2.npz
```

El CLI retorna 0 para CERTIFIED y 2 para REJECTED, guardando el motivo de rechazo en JSON. Si es REJECTED no produce NPZ autorizado para ejecutar.

## Resultado reproducido durante desarrollo

- 13 tests OK, 1 marcado como skipped en el paquete completo.
- Con margen 0.05 m y cinco refinamientos: `certified=true`, costo inicial ≈ 6.05914094, costo final ≈ 5.57754016, `dt_s` ≈ 1.87205723, duración de 19 piezas ≈ 35.56909 s; holgura mínima adicional ante obstáculos ≈ 0.05 m. Las cotas de velocidad ≈ 1.11111 m/s y aceleración ≈ 0.56380 m/s² están bajo los límites elegidos.
- Comparación correcta: el experimento original sin margen extra tenía costo ≈5.55189849 tras cinco refinamientos. Una reserva adicional requiere ceder parte de ese costo.

## Limitaciones para siguiente etapa

La política de WAIT intermedio relajado sigue el criterio del experimento original y depende de certificación de separación sincronizada. El control real con retardo y errores de tracking debe ser validado en la etapa D; si hay sincronización imperfecta, se necesita evaluar también desplazamientos temporales relativos y capacidad de frenado. El validador conservador por envolventes puede rechazar trayectorias físicamente viables.
