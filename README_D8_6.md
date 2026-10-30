# SwarmBots Hybrid V2 — D.8.6, rescate RGB-D de proximidad

## Motivo

Semilla 2026, pickup2, summit0, trash_V1_verde: tras acercarse (depth real 1.253 -> 0.311 m) desaparecieron los pixeles validos de profundidad y el controlador dio `RGBD_V2 TIMEOUT`. La odometria del ultimo mensaje fue (-8.4039648,-19.6754192,-2.4041921) y la pieza objetivo (-8.75,-20), con distancia centro-centro 0.47444 m, offset lateral +0.00760 m y proyeccion de camara +0.12438 m.

## Correccion

- `visual_grasp.py`: se agrega un mecanismo EXPERIMENTAL, opt-in, denominado `near_field_handoff_enabled`. En la configuracion proporcionada queda habilitado solamente para el launch experimental.
- En caso de perder la profundidad, **ordena velocidad cero de inmediato**, nunca un avance inferido. Tras 0.75 s consecutivos sin deteccion, comprueba:
  - distancia longitudinal del centro de la basura entre 0.43 y 0.50 m;
  - desalineacion lateral <= 0.045 m, profundidad geometrica de camara entre 0.055 y 0.16 m;
  - odometria <= 0.50 s;
  - minimo tres medidas RGB-D recientes centradas, tomadas en >= 0.12 s (dentro de los ultimos 3 s), medidas <= 0.46 m y coherentes con proyeccion de camara.
- Solo en esa situacion publica `visual_grasp/result=1`, REGISTRANDO que es una alineacion geometrica PROVISIONAL y NO un joint confirmado. A continuacion, el gestor original de `DetachableJoint` comprueba geometricamente la proximidad y debe confirmar `attached` para que D.7 continue. Si falla el gestor, D.7 se detiene.
- En cualquier otra condicion, el timeout/fallo normal se mantiene: NO autoriza agarres solo por odometria sin historial RGB-D.
- `swarm_cbs_grasp_base.launch.py`: se parte exactamente de D.8.4 (velocidad dinamica y hybrid_enabled), conserva `attach_timeout=5.0` de D.8.1 y habilita el asistente de proximidad para los tres visual_grasp. **No reemplazar con otra version del launch antiguo**.

## Instalacion (host Ubuntu)

```bash
cd ~/swarmbots
cp swarm_ws/src/swarm_behavior/swarm_behavior/visual_grasp.py swarm_ws/src/swarm_behavior/swarm_behavior/visual_grasp.py.bak_D8_6
cp swarm_ws/src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py swarm_ws/src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py.bak_D8_6
unzip -o ~/Descargas/swarm_hybrid_v2_D8_6_rgbd_proximidad.zip -d ~/swarmbots
```

En Docker, con Gazebo detenido:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
python3 -m py_compile src/swarm_behavior/swarm_behavior/visual_grasp.py src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py
colcon build --packages-select swarm_behavior swarm_worlds --symlink-install
source install/setup.bash
python3 scripts/run_swarm_seed_d8.py --seed 2026 --auto-confirm --execute
```

La suite `tests/test_d86_nearfield.py` incluye pruebas independientes de ROS; esta comprobada localmente con `python3 -m unittest discover -s tests -v` desde la carpeta **`~/swarmbots` del host** (si tiene Python y NumPy). No necesitas correrla para instalar el parche.

## Logs importantes

- `RGBD_V2 SIN_DEPTH`: sensor sin pixeles de profundidad suficientes.
- `RGBD_V2 NEAR_FIELD HANDOFF CONTROLADO`: asistente validado (todavia NO significa agarre fisico).
- `JOINT CONFIRMADO`: `grasp_manager` recibio ACK fisico; este evento SÍ es requisito para avanzar.
- `RGBD_V2 TIMEOUT`: sin vision ni confirmacion geometrica valida: fallo conservador.

## Alcance y limites

Esta implementacion no ha sido probada aqui contra Gazebo. Las 10 pruebas aisladas cubren matematicas, condiciones negativas y ausencia de movimiento durante la perdida de profundidad; no garantizan el estado dinamico real ni sustituyen una comprobacion fisica. El asistente es especifico a los parametros del escenario experimental; no debe emplearse como garantia de seguridad en robots reales.

No se han cambiado CBS, las curvas Hybrid V2, limites dinamicos, D.7.3 ni D.8.5, ni la planificacion de oleadas.
