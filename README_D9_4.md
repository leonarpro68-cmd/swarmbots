# SwarmBots D.9.4 — conversión Bézier quíntica offline (NO EJECUTABLE)

## Qué hace

Aproxima cada referencia D.9.2 `B_i(phase_i(t/T_i))` mediante piezas Bézier quínticas temporizadas **independientemente por robot**. Invierte la función de fase para colocar cortes en las uniones de las piezas espaciales originales y subdivide cada pieza (2 cortes por defecto). Cada nueva pieza interpola posición, velocidad y aceleración en los extremos mediante Hermite quíntico.

Comprueba las cotas de velocidad/aceleración de **las curvas convertidas** usando el casco convexo de los polígonos de control derivados y la separación entre robots con un límite Lipschitz entre muestras. Compara errores posición y velocidad con una cota conservadora entre muestras. **El error de aceleración frente a D.9.2 se comprueba solo mediante muestreo, no mediante un certificado global**, por lo que el resultado no autoriza movimiento.

`timed_bezier.v1` requiere un `dt_s` común y no admite las nuevas duraciones individuales. No se generan comandos ROS. Todos los artefactos contienen `NON_EXECUTABLE` o `ros_executable:false`.

## Instalar

Fuera de Docker:

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_D9_4_conversion_offline.zip -d ~/swarmbots
```

Dentro del contenedor Docker (requiere scripts D9, D9.1, D9.2 ya instalados):

```bash
cd ~/swarm_ws
export PYTHONPATH="$PWD/src/swarm_behavior:$PWD/scripts:$PYTHONPATH"
python3 scripts/tests/test_d94_conversion.py
python3 scripts/tests/test_d94_integration.py
```

## Ensayo sobre seed_100_run_007

```bash
python3 scripts/d94_convert_offline.py --run scripts/resultados/d8/seed_100_run_007 --stage pickup2
python3 scripts/d94_convert_offline.py --run scripts/resultados/d8/seed_100_run_007
```

Resultados en `seed_100_run_007/d94_conversion/`:

- `d94_summary.json` — decisión por etapa y comprobaciones.
- `<stage>_conversion_report.json` — cotas y errores detallados por robot y separación entre pares.
- `<stage>_converted_NON_EXECUTABLE.npz` — `R0_controls` `(N,6,2)`, `R0_durations_s` `(N,)` y análogos R1/R2; duración en **segundos** para cada pieza. Incluye parámetros de origen D.9.2.

Si se rechaza por cotas conservadoras, intentar un mayor número de piezas sin modificar los límites de seguridad:

```bash
python3 scripts/d94_convert_offline.py --run scripts/resultados/d8/seed_100_run_007 --subdivisions 4 --step .005
```

No ejecutar ni publicar estos NPZ a los seguidores actuales. La siguiente etapa debe añadir soporte de duración independiente a los mensajes ROS, certificación de errores continuos y de obstáculos, preflight y pruebas en simulación. Los resultados offline D.9.2 no sustituyen los certificados del ejecutor.
