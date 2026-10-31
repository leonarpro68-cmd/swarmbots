D.9.8C.2 — Propuesta de oleadas de entrega (READ ONLY)
========================================================
Este paquete no arranca ROS, no actúa pinzas y no altera misiones. Su misión es preservar la regla original de D8: a lo sumo un robot por depósito por oleada. Al tener dos objetos verdes y uno azul, propone una primera oleada con summit2 (azul) y summit0 (verde), luego una segunda con summit1 (verde).

IMPORTANTE: la propuesta no representa un plan CBS, NO comprueba la geometría de recorridos, la separación con robots estacionados ni certifica la unión de DetachableJoint. La asignación exacta debe verificarse contra el scenario.json vigente de D8 y la instantánea viva de Gazebo. No usar para ordenar movimientos o liberar basuras hasta completar estos gates.

En host: unzip -o ~/Descargas/swarm_D98C2_oleadas_delivery1_preparacion.zip -d ~/swarmbots
En Docker:
cd ~/swarm_ws
python3 -m unittest discover -s scripts/tests -p 'test_d98c2_delivery_waves.py' -v
python3 scripts/d98c2_delivery_waves.py --snapshot scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_live_snapshot.json
Archivo: scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_wave_proposal.json

Siguiente integración necesaria: confirmar uniones reales de carga, estado de misión D8, planificar rutas desde odometría fresca con CBS+D9.2 por oleada teniendo en cuenta robots estacionarios, detener publicadores D8 antes de operar D9.2, ejecutar y confirmar release_result y detach físico antes de avanzar.
