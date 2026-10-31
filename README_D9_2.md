# SwarmBots D.9.2 — reparametrización temporal suave, offline

D.9.2 **no modifica D.8 ni publica comandos ROS**. Analiza las cuatro trayectorias Bézier por etapa provenientes de los ficheros `*_protocol_controls.npz` ya guardados por D.8. Requiere que `d9_offline_retime.py` y `d91_segment_retime.py` estén en la misma carpeta.

## Instalación

Descomprimir el ZIP en `~/swarmbots` fuera del contenedor y ejecutar dentro de Docker:

```bash
cd ~/swarm_ws
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/tests/test_d92_offline.py
```

## Ejecución

```bash
python3 scripts/d92_smooth_retime.py --run scripts/resultados/d8/seed_100_run_007
```

Para experimentar primero con una sola etapa:

```bash
python3 scripts/d92_smooth_retime.py --run scripts/resultados/d8/seed_100_run_007 --stage pickup2
```

Salida: `d92_offline_report.json` dentro de la carpeta de la ejecución.

## Modelo y garantías

Se usan tres parámetros por robot, `T_i`, `a_i` y `b_i`, con progreso normalizado

`u_i(x)= x + a_i sin(2πx)/(2π) + b_i sin(4πx)/(4π)`, `x=t/T_i`.

Así se optimiza la velocidad *dentro* de la ruta, sin cambiar sus puntos de control espaciales. El progreso es monótono, con extremos en 0 y 1. El evaluador calcula velocidad y aceleración mediante la regla de la cadena (incluyendo aceleración tangencial y normal). Rechaza uniones no C2 y velocidad no nula en los extremos. Usa límites conservadores entre muestras obtenidos de cotas sobre el jerk de los polinomios Bézier y la función temporal; para pares de robots emplea la cota Lipschitz entre muestras, contando a los robots ya detenidos en su destino.

Límites: velocidad 1.20 m/s; aceleración 1.00 m/s²; distancia interrobot 1.15 m. El resultado `CERTIFIED_CONSERVATIVE_OFFLINE` solo cubre esas comprobaciones, **no** equivale al certificado completo Hybrid V2 (obstáculos, seguimiento ROS ni operación real). Los resultados de D.9.1 se usan como referencia. Si la comprobación estricta rechaza la referencia, se marca `UNVERIFIED_SMOOTH_BASELINE` en lugar de fabricar mejoras.

La búsqueda es una exploración discreta, no una solución óptima global; puede no mejorar D.9.1. Las pruebas sintéticas no sustituyen ejecutar el análisis sobre datos reales del usuario. No copiar temporizaciones D.9.2 al protocolo de los seguidores D.8: aún no admite duración variable por robot y reparametrizaciones armónicas.
