# D.9.1 — Temporalización offline por bloques Bézier con continuidad C¹

Esta primera iteración es **solo un experimento offline**. No modifica Gazebo, los seguidores, D.8, los límites dinámicos ni los archivos `protocol` existentes.

## Instalación

Desde el HOST, en la raíz del repositorio:

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_hybrid_v2_D9_1_segmentos_offline.zip -d ~/swarmbots
```

Dentro de Docker:

```bash
cd ~/swarm_ws
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/tests/test_d91_offline.py
python3 scripts/d91_segment_retime.py --run scripts/resultados/d8/seed_100_run_007
```

La salida es `seed_100_run_007/d91_offline_report.json`. También incorpora la duración del informe D.9 si existe `d9_offline_report.json` en esa ejecución.

## Método y alcance

- Utiliza los controles quínticos R0, R1 y R2 y dt_s de las cuatro etapas del protocolo D.8.
- Detecta los segmentos con movimiento; las piezas de espera final quedan en su posición final.
- Comprueba la continuidad de posición y tangente Bézier entre segmentos consecutivos. Si la tangente es distinta de cero, ambos segmentos pertenecen a un bloque temporal común. Solo permite cambios de escala entre bloques si ambas tangentes son cero.
- Optimiza de forma reproducible y conservadora un factor de tiempo constante por bloque, con búsqueda local coordinada (no garantiza óptimo global).
- Acota rapidez y aceleración mediante envolventes convexas de las derivadas Bézier y comprueba separaciones mediante muestreo temporal y cota Lipschitz por rapidez máxima. Los robots que terminan permanecen físicamente ocupando su última posición.
- Un rechazo de continuidad produce error y **no** autoriza ejecución. Cuando no se supera el chequeo conservador original, se informa `UNVERIFIED_BASELINE`.

### Precauciones

- Esta versión no crea una temporalización independiente *arbitraria* por segmento: sin detener el robot en los nudos interiores, hacerlo puede introducir discontinuidades de velocidad. Por tanto puede encontrar la misma solución que D.9 o ninguna mejora.
- La derivada de aceleración puede ser discontinua en nudos incluso con velocidad continua; no se certifica jerk ni dinámica de motores.
- `CERTIFIED_CONSERVATIVE_OFFLINE` **no** equivale al certificado espacial de Hybrid V2 ni al preflight de ROS 2. El análisis no genera archivos `protocol` ejecutables.
- El modelo utiliza las curvas planificadas, no odometría de Gazebo. No incluye giros ni aproximaciones RGB-D.
- **No hacer `--execute` ni cargar las duraciones calculadas en los seguidores.** El siguiente paso requerirá un protocolo que admita tiempos por pieza y una nueva certificación integrada.

## Opciones

```bash
python3 scripts/d91_segment_retime.py --run scripts/resultados/d8/seed_100_run_007 \
  --stage pickup2 --max-speed 1.20 --max-accel 1.00 \
  --min-separation 1.15 --step 0.10 --levels 5 --passes 3
```
