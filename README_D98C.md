# D9.8C — Auditoría de tres agarres (solo lectura)

No mueve robots, no acciona pinzas, no toca el registro D.8. Conserva Gazebo y los tres objetos sujetos.

## Instalación

Desde el anfitrión: `unzip -o ~/Descargas/swarm_D98C_auditoria_tres_agarres.zip -d ~/swarmbots`

En Docker: `python3 -m unittest discover -s scripts/tests -p test_d98c_pickup_evidence.py -v`

## Importante: informes sobrescritos

D9.8B.1 usa de manera predeterminada el MISMO destino `d98b/pickup1_turn_visual_grasp.json` en las tres ejecuciones. Si no se usó `--output` distinto ni se guardó una copia, el último informe sobreescribió los anteriores. No se pueden recuperar las dos confirmaciones anteriores a partir del último informe. NO repetir agarres exitosos para recrear informes.

Para auditar, proveer 3 informes originales independientes:

```
python3 scripts/d98c_pickup_evidence.py \
  --handoff scripts/resultados/d9_live/seed_100_pickup1_009/d98a/pickup1_handoff.json \
  --summit0-report /ruta/real/summit0.json \
  --summit1-report /ruta/real/summit1.json \
  --summit2-report /ruta/real/summit2.json
```

Sin los tres informes, mantener el estado en `BLOCKED` sin forzar una confirmación histórica. La observación visual de usuario de los tres agarres sigue siendo información valiosa, pero NO equivale a verificar las tres uniones DetachableJoint. El ACK Int8 tampoco incluye el ID del objeto. Para un compromiso formal del estado de misión, inspeccionar uniones reales de Gazebo y estado transportado por robot.

## Próxima transición

Para `delivery1`, preparar un estado con las tres basuras retiradas del mapa de planificación, carga asignada a cada robot, posiciones actuales de odometría (no las del final de pickup1 antes de RGB-D), CBS/Hybrid V2 desde esas posiciones, D9.2, y una nueva sesión watchdog/ejecutor. NO utilizar `pickup1_009` para `delivery1` ni reiniciar Gazebo antes de guardar la situación si se quiere continuar en el mismo mundo.
