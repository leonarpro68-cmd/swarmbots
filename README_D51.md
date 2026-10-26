# Hybrid V2 — D.5.1: temporalización conservadora afinada

Este parche **no modifica** puntos de control, CBS, SLSQP, el seguidor ni el ejecutor ROS 2. Toma un **plan D.4.1 certificado**, calcula cotas rigurosas de la derivada de las Bézier (subdivisión de de Casteljau en polinomios vectoriales de velocidad y aceleración), y propone una **duración constante por pieza para los tres Summit**. Conserva por ello la sincronización y las curvas geométricas, incluidas sus esperas. La continuidad C² en tiempo físico se conserva al usar un único `dt_s`.

Por qué esto puede acelerar: las cotas normales del polígono de control sin subdividir pueden sobreestimar los máximos reales; subdividir el polinomio derivado reduce el conservadurismo sin recurrir a muestreo como prueba. El tiempo mínimo resultante puede **no disminuir**, si el límite realmente es ajustado. El factor de seguridad inicial se mantiene en 1.08. No hay promesa de duplicar velocidades.

## Instalar (host)

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_hybrid_v2_D5_1_temporalizacion.zip -d ~/swarmbots
```

## En Docker: pruebas y evaluación offline

```bash
cd ~/swarm_ws
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/swarm_behavior:$PYTHONPATH"
python3 -m pytest -q src/swarm_behavior/test/test_hybrid_v2_d51_retime.py
python3 scripts/retime_hybrid_v2_d51.py \
  --report scripts/resultados/d4_live/pickup1_live_report.json \
  --controls scripts/resultados/d4_live/pickup1_live_controls.npz \
  --stage pickup1 --max-speed 1.20 --max-accel 1.00 \
  --safety-factor 1.08 --subdivisions 4 \
  --output-dir scripts/resultados/d51_pickup1
```

La salida muestra `old_total_s`, `new_total_s` y `speed_up`. Genera:

- `scripts/resultados/d51_pickup1/pickup1_retimed_report.json`
- `scripts/resultados/d51_pickup1/pickup1_retimed_controls.npz`

**No ejecutar automáticamente**. Si `speed_up <= 1`, no hubo mejora temporal y vale la pena investigar las limitaciones. Si es >1, repetir el preflight ROS sin `--execute` utilizando los nuevos archivos (con Gazebo reiniciado al escenario correcto y los inicios coincidentes). Antes de mover los robots, verificar mundo/huella y riesgos, y usar exactamente las mismas salvaguardas D.4.2. El certificado geométrico fuente se hereda *solo porque no se cambió ningún punto de control*; las nuevas cotas cinemáticas se vuelven a certificar analíticamente. El seguimiento real, frenado, tiempos de comunicación y obstáculos en Gazebo se deben evaluar otra vez mediante D.5.

No modificar globalmente el valor 0.55 m sin verificar colisiones físicas. No bajar el factor de seguridad ni elevar límites de velocidad de forma arbitraria.
