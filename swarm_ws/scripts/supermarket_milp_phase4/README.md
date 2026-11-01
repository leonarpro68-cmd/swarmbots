# Fase 4 — Integración MILP/CBS con puertas de seguridad de Hybrid V2

**Estado real actual: `BLOCKED_CORRIDOR_GEOMETRY`.** No se han generado controles Hybrid V2 certificados ni relojes V3 de la nueva asignación MILP. Se ha comprobado que las cuatro etapas CBS discreto no superan los corredores físicos del supermercado. Generar NPZ ahora sería peligroso.

## Por qué ocurre

La fase 3 utilizó metas de depósito fijas y un grafo CBS discreto. Las restricciones de Hybrid V2 son más estrictas: obstáculos rectangulares, exclusiones de todas las basuritas aún presentes y marcadores de depósitos; margen centro-estantería 0.60 m, separación entre centros Summit 1.15 m. Por ejemplo, un tramo pickup1 pasa a 0.483 m de un obstáculo con radio de exclusión 0.685 m. El validador reutiliza `corridors_supermarket.py` y el verificador `validar_corredores.py` del paquete original `supermarket_multigoal_hybrid_v2`.

## Instalación (host)

Descomprimir ZIP en Descargas:

```bash
bash ~/Descargas/supermarket_milp_bezier_v3_fase4_v1/install_on_host.sh ~/swarmbots/swarm_ws
```

## Pruebas en Docker — sin Gazebo

```bash
cd ~/swarm_ws/scripts/supermarket_milp_phase4
python3 test_phase4.py
python3 audit_phase4.py --cbs ~/swarm_ws/scripts/supermarket_milp_cbs/results/milp_cbs_live_seed100.json --scenario ~/swarm_ws/config/supermarket/seed_100.json --output /tmp/milp_phase4_geometry_live.json
```

El segundo comando **retorna código 2 intencionalmente si el corredor falla**. Su reporte es informativo y no autoriza ejecución. Si el archivo live no existe, utilizar por defecto las referencias incluidas: `python3 audit_phase4.py --output /tmp/milp_phase4_geometry_reference.json`.

## Próxima corrección necesaria

1. Replanificar MILP→CBS con obstáculos dinámicos y nodos de aproximación a depósitos, no metas proxy arbitrarias.
2. Validar con el mapa geométrico de supermercados y las colisiones de caja 0.09×0.09×0.20; la referencia legacy del corredor aún usa una caja 0.12×0.12 para protección (conservadora, pero debe armonizarse explícitamente).
3. Solo con las cuatro puertas geométricas aprobadas: ejecutar **SLSQP original**, cinco refinamientos y certificado por casco convexo. Cada etapa debe aprobarse antes de producir un NPZ temporal.
4. Ejecutar el **V3 original** sobre controles certificados, comprobar C2 temporal, envolventes 2.0m/s y 2.0m/s² y reloj colectivo. **No reutilizar** los NPZ/clock de la asignación fija.
5. Verificar metas de D8 y continuidad real después de los agarres y descargas.

No se modifican nodos ROS, launch, gripper, CBS existente o Gazebo.
