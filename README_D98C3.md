# D.9.8C.3 — Preflight geométrico de oleadas (SOLO LECTURA)

Este parche verifica la asignación de los depósitos leídos del `scenario.json`, calcula puntos nominales de parada a 0.80 m de cada depósito y examina si los segmentos rectos pasan demasiado cerca de otros robots activos o estacionarios (umbral 1.25 m). Los segmentos NO son rutas CBS y podrían atravesar paredes u obstáculos. Una alarma geométrica significa que se necesita una ruta con desvío, NO que la misión deba abortarse definitivamente. Una aprobación tampoco certifica ausencia de colisiones.

No publica `/cmd_vel`, no mueve pinzas, no modifica el registro D6 ni utiliza Gazebo. Si el escenario no identifica un depósito único para un color, informa `DEPOSIT_NOT_UNIQUE_<color>`. Nunca inventa coordenadas.

Instalación anfitrión: `unzip -o ~/Descargas/swarm_D98C3_preflight_oleadas.zip -d ~/swarmbots`.

Dentro de Docker, con `cd ~/swarm_ws`:

```bash
python3 -m unittest discover -s scripts/tests -p test_d98c3_wave_geometry.py -v
python3 scripts/d98c3_wave_geometry.py \
  --snapshot scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_live_snapshot.json \
  --waves scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_wave_proposal.json \
  --scenario scripts/resultados/d8/seed_100_run_007/scenario.json
```

Salida: `scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_wave_geometry.json`.

**No ejecutar delivery1 todavía.** El pipeline `d96c2b_live_pipeline.py` existente exige D6 `--state` físico para delivery1 y planifica tres robots como una sola etapa; no es un planificador por oleadas con obstáculos estacionarios y descarga intermedia. Para la siguiente integración, adaptar planificador a cada oleada manteniendo estacionarios, obtener evidencia de las uniones y odometría fresca, emitir certificados Hybrid V2 + D9.2 por oleada, detener controladores D8 antes de lanzar seguidor, y verificar liberación antes de iniciar oleada 2.
