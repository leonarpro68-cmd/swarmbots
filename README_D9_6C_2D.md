# D9.6C.2D — Auditoría pasiva de marcos y dinámica

Sin publicadores, sin `/cmd_vel`, sin inyección TF y sin autorización de ejecución.

## Instalar

Desde el HOST Linux en `~/swarmbots`:

```bash
unzip -o ~/Descargas/swarm_D9_6C_2D_auditoria_pasiva.zip -d ~/swarmbots
```

Dentro de Docker (`dev@swarm:~/swarm_ws`):

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/tests/test_d96c2d_frame_audit.py
python3 scripts/d96c2d_frame_dynamics_audit.py \
  --run scripts/resultados/d9_live/seed_100_pickup1_002 \
  --stage pickup1 --observe-seconds 5
```

El script espera que `/clock`, `/summit0/odom`, `/summit1/odom`, `/summit2/odom` estén presentes. No requiere que un controlador publique comandos.

Informe: `RUN/d96c2d_audit/pickup1_frame_dynamics_report.json`.

- `PASS_PASSIVE_AUDIT_FRAMES_UNPROVEN`: el robot estuvo casi inmóvil, el plan sigue coincidiendo con las coordenadas medidas, los marcos nominales son los esperados, el reloj avanzó y la separación real excedió 1.15 m. **No significa que el marco `map` esté calibrado**.
- `REJECT_PASSIVE_AUDIT`: alguna condición falló, revise `details.reasons`.

Los límites por defecto son: deriva 2.5 cm, error de inicio 5 cm, rapidez observada 0.03 m/s, deriva yaw 3°. Duración 5 s, no exige que la captura original se haya hecho en los últimos 120 s porque es una auditoría informativa, **no un prearmado**. Ejecute D9.6C.2C otra vez cuando corresponda.

## Paso pendiente para poder ejecutar

La alineación numérica observada en reposo no determina la transformación de orientación o el origen de cada marco `summitN/odom` con respecto a `map`. Para resolverlo se requiere una fuente independiente de verdad-terreno (por ejemplo, poses globales publicadas por Gazebo y su sincronización con las odometrías) o un árbol TF global correcto proveniente de la simulación. No publicar una transformación TF estática inventada. Además debe añadirse un monitor durante el movimiento y un mecanismo de parada coordinada; estos no están incluidos aquí.

D8 no se modifica. Este ZIP contiene solo el nuevo script, su test y este README. Requiere los scripts D9.6A/D9.6C.2C ya instalados.
