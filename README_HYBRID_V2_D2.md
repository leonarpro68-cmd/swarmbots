# SwarmBots Hybrid V2 — Etapa D.2 (offline)

**Instalación:** el ZIP contiene solo `swarm_ws/scripts/plot_hybrid_v2_mission.py`; requiere los paquetes de D.1 con el parche SLSQP y el `plot_mission_routes.py` original. Descomprimir en `~/swarmbots/` fuera de Docker.

Dentro de Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 scripts/plot_hybrid_v2_mission.py \
  --seed 123 --iterations 2 --margin 0.05 \
  --output scripts/resultados/mision_123_hybrid_v2.png \
  --json-output scripts/resultados/mision_123_hybrid_v2.json \
  --controls-dir scripts/resultados/controles_hybrid_v2
```

El programa retorna **0 si las cuatro etapas tienen `CERTIFIED`** y **2 si alguna está `REJECTED`**, aunque en el segundo caso guarda PNG y JSON. No interpretes el código 2 como error de dibujo. Los paneles rechazados muestran SOLO CBS (punteado), sin curva Bézier. Los `.npz` se exportan exclusivamente para etapas certificadas.

## Secuencia

Se reutilizan los inicios deterministas de `plot_mission_routes.py` y la misma secuencia del `PlanningBridge`:

1. `pickup1`: CBS+V2 contra 6 basuras y depósitos.
2. Se confirma recogida de tres basuras en mapa offline.
3. `delivery1`: CBS+V2 con 3 basuras pendientes.
4. `pickup2`: CBS+V2 con 3 basuras pendientes.
5. Se retiran las tres basuras finales.
6. `delivery2`: CBS+V2 sin basuras pendientes.

Se calcula y certifica **cada etapa antes de mutar el mapa**. Se respetan WAIT finales; WAIT intermedios son libres sujetos a certificación, de acuerdo con la simulación de referencia. La planificación pasa a la siguiente fase a través de los finales **discretos CBS** (coinciden con los extremos Bézier cuando certifican). La secuencia offline no simula pinza ni verifica odometría física.

## Resultado observado seed 123 en este entorno

Con `--iterations 2 --margin .05`:

- `pickup1`: CERTIFIED
- `delivery1`: CERTIFIED
- `pickup2`: REJECTED (par R0-R1, subpaso 0, distancia 1.250 m incompatible con corredor del modelo V2)
- `delivery2`: CERTIFIED

La distancia anterior es una métrica interna de la construcción de corredores, no una medición real de colisión. Antes de ROS 2 debe investigarse el rechazo a partir de geometría, transiciones CBS y radio de exclusión, sin reducir márgenes de seguridad a ciegas.
