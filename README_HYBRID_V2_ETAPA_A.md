# SwarmBots Hybrid V2 — Etapa A (refactorización offline)

Se ha extraído la formulación SLSQP del experimento `experimento_entrega_ronda2_base_ros.zip` sin cambiar la misión ROS 2 existente. Este paquete no publica `/cmd_vel`.

## Qué contiene

- `hybrid_v2/scenario.py`: rutas de referencia de entrega/ronda 2 y `inputs_from_cbs(node_paths,trash,deposits)`, que recibe rutas CBS `(i,j)`; entradas de basura y depósitos en `(posición_métrica, radio_exclusión)`.
- `hybrid_v2/geometry_reference.py`: parámetros geométricos originales copiados para reproducibilidad, sin sustituir `swarm_planning/geometry.py` del proyecto.
- `hybrid_v2/geometry_corridors.py`: funciones geométricas del experimento, aisladas de efectos de ejecución; conserva código legado para comparación, no se recomienda invocar funciones no documentadas.
- `hybrid_v2/slsqp.py`: solver de `ejecutar.py` con eliminación de igualdades por `null_space`, continuidad C2, piezas de espera final constantes y verificación de convergencia.
- `hybrid_v2/refinement.py`: separadores derivados del muestreo de las curvas (Shapely), refinamiento iterativo y rechazo de soluciones que aumenten el costo.
- `hybrid_v2/demo_v2.py`: validación de puntos de control de referencia y ejecución opcional del solver completo.
- `test/test_hybrid_v2.py`: pruebas de geometría, interfaz de entrada y regresión numérica.

## Instalación

En el host, estando en `feature/cbs-hybrid-refinement`:

```bash
cd ~/swarmbots
unzip -o ~/Descargas/swarm_hybrid_v2_etapa_A.zip -d .
```

No se requiere recompilar ningún nodo existente para ejecutar estas pruebas offline. Ejecutar con las dependencias Python del experimento (`numpy`, `scipy`, `shapely`) disponibles y con `PYTHONPATH` apuntando al paquete:

```bash
export PYTHONPATH="$PWD/swarm_ws/src/swarm_behavior:$PYTHONPATH"
python3 -m pytest -q swarm_ws/src/swarm_behavior/test/test_hybrid_v2.py
```

Para comparar exactamente contra el `.npz` del ZIP de referencia, descomprime también el ZIP original y define `HYBRID_V2_REF_NPZ=/ruta/controles_slsqp_refinados.npz` al ejecutar las pruebas.

También puede comprobarse directamente el costo de la solución final existente:

```bash
python3 -m swarm_behavior.swarm_planning.hybrid_v2.demo_v2 \
  --reference-npz /ruta/controles_slsqp_refinados.npz
```

Para activar el cálculo SLSQP completo sobre las rutas de referencia añada `--solve`. Es una prueba CPU intensiva y puede tardar bastante; no se ejecutó hasta el final en esta entrega.

## Límites y siguientes pasos

1. **La validación demostrada es parcial**: tres pruebas de integración básica y una regresión de costo. No se ha reproducido aquí el ciclo completo de 5 iteraciones ni certificado el mapa/colisiones de Gazebo.
2. `inputs_from_cbs` recibe rutas completas del CBS **pero aún no está conectado al planificador de las cuatro fases ni a ROS 2**. En la etapa B se hará una regresión completa sobre entrega/ronda 2 con tolerancias para costos y restricciones.
3. Sigue sin haber escalamiento temporal físico ni seguimiento. Es trabajo de las etapas C/D.
4. El paquete mantiene como referencia la geometría del ZIP y sus **radios de exclusión**; no inflar dos veces los depósitos, basuras o columnas.
5. `geometry_corridors.py` conserva código antiguo que no es utilizado por la API V2 estable. En un endurecimiento posterior conviene extraer únicamente las primitivas usadas y eliminar implementaciones duplicadas.
