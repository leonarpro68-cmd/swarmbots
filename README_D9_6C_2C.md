# D.9.6C.2C — prearm sombra ROS 2 (NO EJECUTABLE)

Este parche agrega **solo** `swarm_ws/scripts/d96c2c_prearm_readonly.py` y `swarm_ws/scripts/tests/test_d96c2c_prearm.py`. No modifica D.8 ni otros nodos. No crea publishers, no emite `/cmd_vel`, no inyecta TF y no autoriza movimiento.

El verificador compara los puntos iniciales del plan D.9.4 con la captura que utilizó D.9.6C.2B, verifica SHA-256 de los archivos del escenario y controles y lee el manifiesto certificado por D.9.5. Luego observa odometría de los tres Summit y `/clock`. Rechaza si alguna posición difiere más de 5 cm, si algún Summit tiene una rapidez mayor de 0.03 m/s, si falta odometría, si los marcos identificados son inesperados, si el reloj no avanza o si la captura es anterior a 120 s de simulación. Ese límite es configurable; no es una autorización para reutilizar planes obsoletos. **La equivalencia TF entre `map` y `summitN/odom` sigue sin certificarse.**

## Instalación

Fuera de Docker:

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_D9_6C_2C_prearm_sombra.zip -d ~/swarmbots
```

Dentro de Docker:

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/tests/test_d96c2c_prearm.py
```

Con el mismo Gazebo que contiene los tres Summit inmóviles y puentes `/clock`, `/summit0/odom`, `/summit1/odom`, `/summit2/odom`, inspeccione primero que recibe los tópicos y use la carpeta REAL que produjo D.9.6C.2B:

```bash
python3 scripts/d96c2c_prearm_readonly.py \
  --run scripts/resultados/d9_live/seed_100_pickup1_001 \
  --stage pickup1
```

Salida: `d96c2c_prearm/pickup1_prearm_report.json`. `PASS_SHADOW_PREARM_ONLY` es válido exclusivamente para esta comprobación pasiva. `REJECT_STALE_OR_RESET_CAPTURE` puede ocurrir si ha transcurrido mucho tiempo desde la captura original; en tal caso vuelva a ejecutar D.9.6C.2B en una **carpeta nueva**, antes de D.9.6C.2C.

La validación de la orientación global, correspondencia de marcos a lo largo del movimiento, parada coordinada y supervisión de seguimiento **siguen pendientes** antes de conectar publicadores de velocidad.
