# D.9.8C.8 — Auditoría conservadora de mapa previa a Hybrid V2

Esta entrega **NO es todavía la integración/certificación Hybrid V2**: es un preflight que analiza la envolvente convexa de cada segmento Bézier de las tres fases respecto a los obstáculos del mapa vectorial y sus límites. No publica comandos, no lanza Gazebo y no cambia D.8. Por seguridad falla si se elige un mapa incompatible. Antes de ejecutar Hybrid V2 se necesita el mapa que corresponda realmente al escenario D.8; el mapa `colored_warehouse_vector.yaml` original puede no cubrir la zona de entrega azul.

Requisitos Docker: Python, numpy, PyYAML y shapely (`python3 -c "import yaml,numpy,shapely"`).

Instalar ZIP desde el equipo anfitrión: `unzip -o ~/Descargas/swarm_D98C8_auditoria_mapa_hybrid.zip -d ~/swarmbots`

Prueba: `python3 -m unittest discover -s scripts/tests -p test_d98c8_map.py -v`

Ejecución (sin Gazebo, desde `~/swarm_ws`):

```bash
python3 scripts/d98c8_hybrid_map_audit.py \
 --preview scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_bezier_three_phases_preview.json \
 --retimed scripts/resultados/d9_live/seed_100_pickup1_009/d98c1/delivery1_d92_three_phases_offline.json \
 --map-vector src/swarm_worlds/maps/colored_warehouse_vector.yaml
```

Si esa ruta no es el mapa correcto, indicar **el mapa vectorial real** del mundo D.8 con `--map-vector RUTA`. La salida se guarda como `delivery1_d98c8_hybrid_map_audit.json`. Una respuesta `BLOCKED_MAP_GEOMETRY` es un hallazgo válido, no la autorización para modificar los objetivos o ignorar colisiones.

Resultados:
- `MAP_HULL_SCREEN_PASS_NOT_HYBRID_CERTIFIED`: las envolventes conservadoras están dentro del mapa y lejos de obstáculos declarados; NO es certificación Hybrid V2.
- `BLOCKED_MAP_GEOMETRY`: incompatibilidad de mapa o trazado. Puede ser conservadurismo de convex hull o falta de cobertura; investigar antes de seguir.

Pendiente para D.9.8C.9: identificar el mapa D.8 real, replanificar o suavizar con Hybrid V2 verificando obstáculos reales; D9.4/D9.5; cargas y odometría vivas; liberación y nueva planificación entre oleadas.
