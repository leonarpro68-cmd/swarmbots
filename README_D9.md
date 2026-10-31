# SwarmBots D.9 — temporalización independiente offline (prototipo seguro)

Este paquete **NO modifica ningún archivo existente**, no publica ROS y no modifica los controladores. Instala `swarm_ws/scripts/d9_offline_retime.py` y su prueba.

## Instalación

Desde host, copiar el ZIP a Descargas y:

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_hybrid_v2_D9_temporalizacion_offline.zip -d ~/swarmbots
```

Desde Docker:

```bash
cd ~/swarm_ws
python3 -m py_compile scripts/d9_offline_retime.py
python3 scripts/tests/test_d9_offline.py
python3 scripts/d9_offline_retime.py --run scripts/resultados/d8/seed_100_run_007
```

Por defecto, procesa las etapas pickup1, delivery1, pickup2 y delivery2, y guarda `seed_100_run_007/d9_offline_report.json`.

Para una sola etapa: `--stage pickup2`. Para mayor resolución de verificación: `--time-step 0.05`. Parámetros predeterminados: `--max-speed 1.20 --max-accel 1.00 --min-separation 1.15 --grid-steps 5`.

## Qué calcula

Lee las Bézier quínticas de `mission/<etapa>/protocol/<etapa>_protocol_controls.npz`, así como el `dt_s` ejecutado. Para cada robot localiza hasta qué pieza tiene movimiento (el resto se considera espera) y asigna un *factor constante individual* a la duración de su movimiento activo. Una vez que termina, permanece en la última posición hasta el final de la etapa. Busca tres factores por enumeración finita reproducible, con el objetivo de minimizar el tiempo de llegada del último robot.

**Verificación conservadora**:

* Velocidad y aceleración mediante cotas superiores de los puntos de control de la derivada Bézier, ajustados por los factores temporalizados; si se excede el límite, descarta el candidato.
* Distancia entre cada par en pasos de tiempo y cota de desplazamiento Lipschitz entre muestras: `d_cert >= d_muestra - (vmax_i+vmax_j)*h/2` donde `h` es el intervalo temporal. Exige `d_cert >= 1.15 m` durante toda la etapa, incluidas esperas.
* No oculta que la muestra conservadora puede rechazar también trayectorias que el certificador original aceptó, por ser una prueba diferente y más restrictiva.

## Estados del reporte

* `CERTIFIED_CONSERVATIVE`: temporalización evaluada con cotas conservadoras de cinemática y separación entre **robots**. No equivale a una certificación completa Hybrid V2: las distancias a obstáculos y condiciones reales del controlador deben revalidarse antes de ejecutar.
* `UNVERIFIED_BASELINE`: incluso la trayectoria de referencia no supera este nuevo criterio conservador; **NO** se recomienda modificar tiempos ni ejecutar el candidato. No significa que D.8 sea inseguro; sus certificados son distintos.

**No se implementa todavía la temporalización general `s_i(t)` por segmentos, ni se exporta protocolo ejecutable**. La fase siguiente, tras ver los reportes de varias semillas, será optimizar duraciones por segmento preservando límites y las separaciones certificadas, con validación dinámica y pruebas en Gazebo.

No modifica ni reemplaza D.8, D.8.11, sus ficheros de resultados o la evidencia de odometría.
