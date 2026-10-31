# D.9.8C.1: instantánea en vivo de cargas y posiciones antes de delivery1

Este parche es **solo lectura**: NO mueve robots, no acciona pinzas, no cambia el ledger y NO ejecuta ni planifica delivery1. No se repiten agarres exitosos. Requiere `d98b_grasp_preflight.py` instalado desde D.9.8B.

En el host:

```bash
unzip -o ~/Descargas/swarm_D98C1_snapshot_delivery1.zip -d ~/swarmbots
```

Dentro del contenedor (Gazebo activo, objetos sujetos, robots quietos):

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/scripts:$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m unittest discover -s scripts/tests -p test_d98c1_delivery_snapshot.py -v
python3 scripts/d98c1_delivery_snapshot.py \
 --handoff scripts/resultados/d9_live/seed_100_pickup1_009/d98a/pickup1_handoff.json \
 --world world_demo
```

Informe: `scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_live_snapshot.json`.

Un `LOAD_GEOMETRY_COMPATIBLE_NOT_JOINT_CERTIFIED` significa solamente que las tres basuras están cerca de las pinzas y los robots quietos. Los ACK históricos de Summit1/2 se sobreescribieron. `grasp_manager` almacena internamente `self.held` y recibe confirmación `attached` desde Gazebo, pero no publica el identificador de carga como estado persistente. La instantánea no certifica DetachableJoint ni autoriza entrega; se necesita consulta de unión física fiable o telemetría read-only del manager y ledger antes de lanzar D9.2 de delivery1. No copiar el estado de pickup1 a la misión como hecho certificado.
