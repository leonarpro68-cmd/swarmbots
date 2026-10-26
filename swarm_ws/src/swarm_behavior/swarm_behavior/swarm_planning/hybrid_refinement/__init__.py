"""Postprocesado de trayectorias de CBS (experimental, offline).

El adaptador no altera CBS, no publica comandos ROS y NO constituye
una trayectoria dinámica validada. Fase 3 ofrece curvas Bézier offline, pero no son órdenes ROS ejecutables.
"""

from .adapter import (CBSAdapterError, DiscreteSchedule, TimedStep,
                      from_cbs_paths)

from .corridors import (CorridorError, HybridCorridorPlan, SafeCorridor,
                        build_hybrid_corridors)

__all__ = ['CBSAdapterError', 'DiscreteSchedule', 'TimedStep', 'from_cbs_paths',
           'CorridorError', 'HybridCorridorPlan', 'SafeCorridor', 'build_hybrid_corridors']

# SciPy solo se importa al invocar bezier_optimizer: las Fases 1 y 2
# siguen funcionando sin las dependencias numéricas opcionales de Fase 3.
