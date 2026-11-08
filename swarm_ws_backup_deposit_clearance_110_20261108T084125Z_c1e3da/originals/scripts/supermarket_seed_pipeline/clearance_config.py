"""Exclusiones de Hybrid V2 de supermercado, distintas del margen del grafo CBS.

R_SHAPE_M = 0.55 m: círculo de planificación adoptado como hipótesis;
FREE_GAP_M = 0.05 m: separación física deseada.

El grafo A*+CBS sigue usando su clearance antiguo de 0.936 m.
La verificación ROS/Gazebo debe confirmar que el círculo 0.55 m
encierra toda la colisión del Summit (y de cualquier carga transportada).
"""
SUMMIT_RADIUS_M = 0.55
FREE_GAP_M = 0.05
RECTANGLE_CENTER_CLEARANCE_M = SUMMIT_RADIUS_M + FREE_GAP_M
WALL_CENTER_CLEARANCE_M = SUMMIT_RADIUS_M + FREE_GAP_M
INTERROBOT_CENTER_SEPARATION_M = 2 * SUMMIT_RADIUS_M + FREE_GAP_M
# El separador original utiliza dos semiplanos a distancia R del punto medio.
INTERROBOT_HALFSEPARATOR_M = INTERROBOT_CENTER_SEPARATION_M / 2.0
assert abs(RECTANGLE_CENTER_CLEARANCE_M-0.60) < 1e-12
assert abs(INTERROBOT_CENTER_SEPARATION_M-1.15) < 1e-12

# Caja de basura: 0.09 x 0.09 m (SDF sim_summit_supermarket.launch.py).
# El centro de la caja está protegido por su semidiagonal más el radio Summit
# y los 5 cm de holgura de carrocería.
import math
TRASH_BOX_HALF_DIAGONAL_M = math.hypot(0.09, 0.09) / 2
TRASH_CENTER_EXCLUSION_M = SUMMIT_RADIUS_M + FREE_GAP_M + TRASH_BOX_HALF_DIAGONAL_M
# Depósitos: marcadores visuales SIN colisión SDF; mantener política virtual
# previa de navegación cuando no sean origen/destino de una etapa.
VISUAL_DEPOSIT_NAV_EXCLUSION_M = 0.50
