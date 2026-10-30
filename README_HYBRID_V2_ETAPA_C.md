# Hybrid V2 — Etapa C: preflight de seguridad y temporización (offline)

Esta entrega **no cambia nodos, launch ni controladores ROS 2**. Se añade `hybrid_v2/safety.py` y una utilidad `demo_safety.py`. Se mantienen íntegros los módulos A y B.

## Contrato de validación

- Se reciben controles Bézier `N × K × 6 × 2`, posiciones CBS `N × (K+1) × 2` y rutas discretas originales (sin padding).
- Cada pieza debe respetar los límites del warehouse y las zonas de exclusión circulares, **más una reserva operacional adicional** (por defecto 0.05 m). Los radios de obstáculos recibidos se consideran ya inflados al centro del robot: no se agrega un segundo radio físico.
- Para garantizar todo el intervalo, la distancia se calcula sobre la **envolvente convexa de los seis puntos de control**, no por muestreo. Esto es conservador: una pieza Bézier válida podría rechazarse por la amplitud de su envolvente.
- Se exige distancia interrobot adicional de 0.05 m además de `2 * CLEARANCE_WALL_M` entre envolventes de piezas del mismo intervalo.
- Continuidad C², extremos fijos, esperas CBS internas como inmovilidad estricta y permanencia fija después de la meta.
- Se calculan cotas analíticas conservadoras mediante diferencias de control Bézier (velocidad `5(Pj+1 - Pj)` y aceleración `20(Pj+2-2Pj+1+Pj)`). Todos usan un `dt_s` común con factor temporal >=1.
- Si no se cumplen las condiciones, se lanza `ValueError` y **NO** se publica ninguna trayectoria. En la integración futura, el coordinador decidirá cómo replanificar/fallback, sin ejecutar automáticamente trayectorias que no tengan certificado.

**Importante:** el solucionador SLSQP de V2 no impone todavía WAIT intermedios. Si el CBS los necesita y la solución se mueve dentro del intervalo, el validador la rechaza. La siguiente versión puede incorporar esas igualdades al optimizador o separar intervalos. Se mantiene el compromiso de no reinterpretar silenciosamente un WAIT como movimiento.

**Alcance:** se certifica un modelo 2D de obstáculos circulares y robots sincronizados. No certifica física Gazebo, errores reales de odometría, latencia ni la totalidad de una geometría SDF arbitraria.

## Instalación

En Ubuntu, desde el repositorio: `unzip -o ~/Descargas/swarm_hybrid_v2_etapa_C_completo.zip -d ~/swarmbots`.

Dentro del contenedor:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m pytest -q src/swarm_behavior/test/test_hybrid_v2.py src/swarm_behavior/test/test_hybrid_v2_stage_b.py src/swarm_behavior/test/test_hybrid_v2_stage_c.py
```

Probar el certificado sobre las curvas de referencia guardadas:

```bash
python3 -m swarm_behavior.swarm_planning.hybrid_v2.demo_safety \
  --reference-dir "$PWD/src/swarm_behavior/test/data/hybrid_v2_reference" \
  --output /tmp/hybrid_v2_safety.json \
  --operational-margin 0.05 \
  --max-speed 1.20 --max-accel 1.00
```

**Un rechazo es un resultado esperado y útil**, no un fallo de pytest. Las trayectorias del experimento tenían holguras muestreadas de unos 0.026 m y 0.036 m para R0 y R1. Un margen operacional >=0.05 m puede exigir modificar las curvas antes de ejecutarlas en ROS. El script sale con código 2 si no certifica la solución.
