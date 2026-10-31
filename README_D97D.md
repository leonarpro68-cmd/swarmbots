# D.9.7D — Auditoría de seguimiento de pickup1 (solo lectura)

Instalar desde anfitrión: `unzip -o ~/Descargas/swarm_D97D_auditoria_pickup1.zip -d ~/swarmbots`.

Dentro de Docker, en `~/swarm_ws`:

```bash
python3 -m unittest discover -s scripts/tests -p test_d97d_evaluate.py -v
python3 scripts/d97d_evaluate.py \
 --run scripts/resultados/d9_live/seed_100_pickup1_009 \
 --stage pickup1 \
 --relay-report scripts/resultados/d9_live/d97c1_relay_full_002.json
```

Se creará `.../seed_100_pickup1_009/d97d/pickup1_evaluation.json`.

## Limitaciones de medición

El CSV de D.9.7C.1 contiene solamente `t_s`, **máximos acumulados** de error de posición y separación muestreada. No contiene velocidades instantáneas, aceleraciones, yaw, poses globales completas ni tiempos reales de llegada a la meta. No calcules RMSE de errores a partir de máximos acumulados. El campo `horizon_s` es tiempo planificado, no tiempo real medido. Esta auditoría valida consistencia con el reporte y relay, no la actuación física ni la entrega de basura.

Para próximas ejecuciones, agregar registro por robot de `clock_s`, odom pose XY/yaw, velocidad XY, comando de velocidad, referencia XY/velocidad y eventos del relay. Sin cambiar límites o activar movimiento en esta etapa.
