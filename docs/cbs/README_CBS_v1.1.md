# SwarmBots — misión física multiagente A* + CBS (v1.1)

> **Estado:** misión automática funcional en ROS 2 Jazzy y Gazebo Harmonic. Esta guía complementa el `README.md` general de SwarmBots y la documentación histórica de CBS. La ejecución con una sola terminal se probó con `seed=123`; la ejecución supervisada previa se probó con varias semillas. La posición final dentro de cada depósito **no se verifica automáticamente**.

## ¿Qué demuestra este módulo?

Tres robots **Summit XLS omnidireccionales**, cada uno con pinza y cámara RGB-D, realizan una misión de clasificación de **seis basuras** (dos azules, dos verdes y dos naranjas) en **tres depósitos** de colores correspondientes. La misión se desarrolla en **dos rondas** de tres recogidas y entregas. El sistema combina:

- **A\*** para obtener caminos individuales sobre una representación discreta del entorno.
- **CBS (Conflict-Based Search)** para coordinar caminos de los tres robots y resolver conflictos del modelo de planificación.
- **Ejecución por pasos con barreras** para sincronizar a los robots durante la recogida y el transporte.
- **RGB-D** para orientar y alinear cada robot con su basura al final de la trayectoria.
- **Pinza física y `DetachableJoint`** de Gazebo para sujetar y soltar objetos, esperando confirmaciones de agarre y liberación.
- **Entregas en oleadas** para evitar que dos robots intenten utilizar simultáneamente un mismo depósito.

El sistema v1.1 automatiza el arranque de Gazebo, el bridge del reloj y las transiciones de las dos rondas. **No incorpora recuperación autónoma de errores** ni sustituye las comprobaciones físicas por supuestos.

## Inicio rápido — una sola terminal

**Ejecutar desde Ubuntu (host), fuera de Docker:**

```bash
cd ~/swarmbots
./run_swarmbots.sh --seed=123
```

El script usa el contenedor existente **`swarm_dev`**, que **debe estar en ejecución**. Ya **no es necesario entrar manualmente en Docker** ni abrir terminales separadas para Gazebo, `/clock` y la misión. Si el contenedor está apagado:

```bash
docker start swarm_dev
```

Para comprobarlo:

```bash
docker ps --filter name=swarm_dev
```

Para ejecutar otra distribución de objetos:

```bash
./run_swarmbots.sh --seed=456
```

El lanzador comprueba la presencia del mundo base, genera un escenario reproducible con la semilla, prepara el archivo SDF, inicia Gazebo y el bridge `/clock`, espera a que lleguen el reloj y las tres odometrías y ejecuta la misión completa.

**Al terminar:** Gazebo permanece abierto para inspeccionar las seis basuras en los depósitos. **Al fallar:** la misión se detiene y Gazebo también debe permanecer abierto para diagnosticar el estado físico, según el ajuste final de v1.1. Para cerrar la simulación, pulsa **`Ctrl+C`** en la terminal del lanzador. No reanudes ni relances una misión fallida sobre ese mismo estado del mundo.

La opción `--close-on-success` permite usar la política antigua de cierre después del éxito:

```bash
./run_swarmbots.sh --seed=123 --close-on-success
```

> **Precaución:** `run_swarmbots.sh` no arranca por sí solo un contenedor detenido. Se requiere el entorno ROS 2 ya compilado dentro de `swarm_dev` y los paquetes correctamente instalados. El launch necesita acceso al servidor gráfico/GPU según la configuración Docker existente.

## Flujo de ejecución

```text
Ubuntu host: ./run_swarmbots.sh --seed=N
                 |
                 v
Docker swarm_dev: run_swarmbots_auto.py
                 |
      +----------+-----------+
      |                      |
  Escenario JSON         Mundo Gazebo SDF
      |                      |
      +----------+-----------+
                 v
      Gazebo + bridge /clock
                 |
       Espera /clock y odometría
                 |
                 v
  Coordinador automático de dos rondas
                 |
       +---------+---------+
       |                   |
   RONDA 1             RONDA 2
       |                   |
  CBS recogida         CBS recogida
       |                   |
  RGB-D + attach ACK   RGB-D + attach ACK
       |                   |
  CBS transporte       CBS transporte
       |                   |
  Entrega por oleadas  Entrega por oleadas
  + detach ACK         + detach ACK
       |                   |
  Regreso al vértice   Regreso al vértice
       +---------+---------+
                 |
         6 agarres + 6 detach
                 |
       Gazebo queda abierto
       hasta Ctrl+C
```

Las transiciones **no piden** `INICIAR`, `ENTREGAR`, `CONTINUAR` ni `RONDA2`. Sin embargo, **sí exigen** que los ejecutores y controladores comuniquen el resultado previsto en cada fase. Si una confirmación crítica falla, no se continúa la misión.

## Archivos principales

Las rutas son relativas a la raíz del repositorio:

| Función | Ruta |
|---|---|
| Lanzador desde Ubuntu | `run_swarmbots.sh` |
| Orquestación de Gazebo, `/clock` y misión | `swarm_ws/scripts/run_swarmbots_auto.py` |
| Generador de escenarios | `swarm_ws/scripts/escenario_aleatorio_swarm.py` |
| Preparación del mundo con grippers/objetos | `swarm_ws/scripts/swarm_cbs_grasp_prepare.py` |
| Launch de la simulación CBS | `swarm_ws/src/swarm_worlds/launch/swarm_cbs_grasp_base.launch.py` |
| Coordinador automático integral | `swarm_ws/scripts/mision_fisica_completa_auto.py` |
| Coordinadores automáticos por ronda | `swarm_ws/scripts/mision_fisica_ronda1_auto.py`, `swarm_ws/scripts/mision_fisica_ronda2_auto.py` |
| Entregas por oleadas | `swarm_ws/scripts/entregas_paralelas_cbs_auto.py` |
| Recogida CBS ronda 1 | `swarm_ws/scripts/ros_recogida_cbs_v2.py` |
| Recogida CBS ronda 2 | `swarm_ws/scripts/ros_recogida_ronda2_cbs_v2.py` (con mejoras de diagnóstico/reenvío V3) |
| Transporte CBS | `swarm_ws/scripts/ros_transporte_fisico_cbs_v2.py`, `swarm_ws/scripts/ros_transporte_ronda2_cbs_v2.py` |
| Alineación RGB-D | `swarm_ws/src/swarm_behavior/swarm_behavior/visual_grasp.py` |
| Gestión de pinza y joint | `swarm_ws/src/swarm_behavior/swarm_behavior/grasp_manager.py` |
| Validación por semillas, modo anterior | `swarm_ws/scripts/validar_semillas_swarm.py` |

Los nombres de scripts con sufijos `v2`/`v3` son históricos. La funcionalidad documentada se corresponde con la rama `feature/swarmbots-auto-v1.1` publicada por el desarrollador; verifica el commit o etiqueta exacta de tu checkout al reproducir un experimento.

## ROS 2 y Gazebo

Se utilizan **ROS 2 Jazzy** y **Gazebo Harmonic** en el workspace `swarm_ws`. Los paquetes identificados en el repositorio incluyen `swarm_behavior`, `swarm_worlds`, `summit_xl_description`, `summit_xl_control`, `swarm_description`, `robotnik_common` y `robotnik_sensors`.

El flujo de ejecución usa las odometrías `/summit0/odom`, `/summit1/odom` y `/summit2/odom`, el reloj `/clock`, objetivos de navegación, el control visual y las interfaces de agarre/liberación de cada robot. El lanzador administra los procesos de simulación; los controladores físicos siguen siendo independientes de A* y CBS.

## Validación funcional registrada

| Semilla | Modalidad / configuración | Resultado conocido | Duración total reportada |
|---|---|---|---:|
| `123` | Misión física inicial supervisada | 6 ACK de agarre y 6 de liberación | No registrada |
| `456` | Controladores originales | Fallo de agarre/alineación en segunda ronda | 270,27 s hasta el fallo |
| `456` | Controladores físicos V2 | 6 ACK de agarre y 6 de liberación | 451,37 s |
| `789` | Ejecutores anteriores | Timeout de `STEP_1` en segunda ronda | 262,33 s hasta el fallo |
| `789` | Controladores V2 + ejecutor CBS V3 | 6 ACK de agarre y 6 de liberación | 440,54 s |
| `2026` | Controladores V2 + ejecutor CBS V3 | 6 ACK de agarre y 6 de liberación | 584,70 s |
| `123` | **v1.1 automática, una terminal** | **6 ACK de agarre y 6 de liberación; sin preguntas manuales** | No registrada |

**Interpretación:** son ensayos de validación funcional, realizados con versiones y modalidades diferentes. No constituyen una estimación estadística de la tasa de éxito. Los tiempos de las pruebas supervisadas incluyen pausas de autorización humana y no son comparables directamente con una ejecución desatendida. Los ACK no demuestran por sí solos que cada objeto terminó dentro del área correcta del depósito.

## Resultados y diagnóstico

Las ejecuciones automáticas guardan datos en un directorio exclusivo por intento, similar a:

```text
/tmp/swarm_autonomo_v11/seed_123_YYYYMMDD_HHMMSS_<id>/
├── scenario.json
├── world.sdf
├── mission_ronda1_estado.json
├── mission_ronda2_estado.json
└── ... registros de ejecución ...
```

La ruta exacta aparece en la consola al ejecutar la misión. Esos archivos se guardan **dentro del contenedor** y pueden perderse si se elimina o recrea. Consérvalos fuera del contenedor cuando sean necesarios para análisis o demostraciones.

Si una ronda falla, **no se inicia automáticamente la siguiente**. El entorno queda disponible para observar posiciones y joints, pero no debe reanudarse una misión parcial sobre el mismo Gazebo. Para un nuevo intento, cierra la simulación e inicia una instancia limpia.

## Limitaciones y trabajo futuro

- La misión funciona sin confirmaciones manuales, pero requiere un contenedor Docker previamente iniciado.
- Una ejecución con seis ACK completos **no equivale** a verificar automáticamente que las seis basuras estén en los depósitos correctos.
- Los conflictos que resuelve CBS corresponden al modelo discreto de planificación; no constituyen una garantía absoluta de seguridad física durante el seguimiento continuo.
- No hay recuperación automática general ante fallos de cámara, agarre, joint o navegación; la estrategia sigue siendo detenerse y conservar el estado para inspección.
- Los resultados disponibles no acreditan aún robustez estadística ni escalabilidad a más de tres robots.

## Documentación relacionada

- [`README.md`](../../README.md): presentación general del proyecto, otros mundos, ROS 2, Gazebo, teleoperación y trabajo relacionado con ACT.
- [`mission_cbs_v1.md`](mission_cbs_v1.md): procedimiento **supervisado** de v1.0 y antecedentes de las pruebas.
- [`arquitectura_cbs_v1.md`](arquitectura_cbs_v1.md): arquitectura detallada de la versión CBS anterior.


