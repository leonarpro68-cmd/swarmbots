# SwarmBots D.9.3 / D.9.4 — vista previa offline segura

Esta entrega implementa **D.9.3** (comparaciones gráficas, CSV de referencia y
revalidación independiente de los parámetros guardados) y el **primer análisis
de compatibilidad de D.9.4**. NO implementa aún conversión certificada ni
publica comandos ROS. No modifica código existente de la misión.

## Instalación (fuera del contenedor)

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_D9_3_D9_4_preview_seguro.zip -d ~/swarmbots
```

## Dentro del contenedor

```bash
cd ~/swarm_ws
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/tests/test_d93_preview.py
python3 scripts/d93_validate_preview.py --run scripts/resultados/d8/seed_100_run_007
```

Los archivos quedan en `seed_100_run_007/d93_preview/`:
- `pickup1_d8_vs_d92.png`, etc. (seis paneles por etapa)
- `pickup1_reference_NON_EXECUTABLE.csv`, etc. (muestras p/v/a)
- `pickup1_audit.json`, etc. y `d93_summary.json`

**Importante:** `timed_bezier.v1` del adaptador D.3 actual describe curvas
Bézier quínticas de duración `dt_s` **común para todos los robots**. No
transporta una función de fase no lineal arbitraria para cada Summit. Por
ello no podemos simplemente sustituir los campos `dt_s` y afirmar que ROS
seguirá exactamente D.9.2.

Próximo hito técnico D.9.4: un aproximador de la trayectoria cartesiana
p_i(t) a curvas quínticas de tiempos compatibles, con verificación continua
de error geométrico y cinemático, preservación de paradas e inicio sincronizado,
re-certificación contra obstáculos y entre robots, y un adaptador/follower
compatible con duraciones distintas. Alternativamente, extender el follower
para evaluar la fase armónica directamente. Hasta entonces, todo lo aquí
generado es **SOLO REFERENCIA OFFLINE**.
