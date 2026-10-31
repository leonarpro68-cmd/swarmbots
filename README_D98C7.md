# D.9.8C.7 — Modelo temporal D.9.2 de las tres fases (OFFLINE)

Este parche **no altera** D.8 ni ningún ejecutor. Incluye `d98c7_phase_retime.py` y `d98c7_d92_verifier.py`: el segundo adapta `phase`, `values_for_robot` y `verify` del D.9.2 original para validar curvas quinticas con limites conservadores de velocidad, aceleracion y separacion. Se aplica solamente un factor de escala **uniforme** a la duracion de cada fase (armonica a=b=0). No es la busqueda completa independiente de D.9.2; tampoco sustituye Hybrid V2 ni la certificacion D.9.4/D.9.5.

## Instalacion
En host:
```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_D98C7_temporalizacion_offline.zip -d ~/swarmbots
```
En Docker:
```bash
cd ~/swarm_ws
python3 -m unittest discover -s scripts/tests -p 'test_d98c7.py' -v
```

## Analisis
Necesita la salida del convertidor D.9.8C.6 (JSON y NPZ):
```bash
python3 scripts/d98c7_phase_retime.py \
 --preview scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_bezier_three_phases_preview.json
```
Crea `delivery1_d92_three_phases_offline.json`. Exito esperado: `D92_PHASE_MODEL_THREE_PHASES_PASS_NOT_EXECUTABLE`. Si archivo D.9.8C.6 no existe, generar primero como se indico en esa etapa.

## Limitaciones bloqueantes
- Todas las posiciones provienen de snapshot archivado anterior al cierre de Gazebo.
- Son curvas de linea recta entre nodos CBS discretos; no existe aun certificacion de obstaculos Hybrid V2.
- No se han recreado uniones de carga ni comprobado liberaciones entre fases.
- Solo verifica temporalizacion escalar con el verificador de D.9.2; no optimiza coeficientes armonicos ni demuestra mejora respecto a D.8.
- No emite ROS ni autorizacion de ejecucion; no use los controles de este informe como comandos.

Paso siguiente: D.9.8C.8, integrar trazado continuo obstaculo-seguro Hybrid V2, despues planificar/validar desde estado restaurado, con chequeos D.9.4/D.9.5 por fase y liberaciones D.8.
