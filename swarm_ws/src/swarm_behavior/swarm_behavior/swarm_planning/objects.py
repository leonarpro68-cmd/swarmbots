"""Definiciones de objetos fijos, sin dependencias de Gazebo."""
from dataclasses import dataclass

COLORS = {
    'verde': (0.18, 0.72, 0.28),
    'naranja': (0.98, 0.46, 0.11),
    'azul': (0.12, 0.35, 0.94),
}

# Coordenadas de vértices (i,j) de nuestro grafo compartido de 1.25 m.
# Los primeros seis son zonas de basura bloqueables; los últimos tres,
# depósitos asociados al color. Se eligen alejados de columnas.
@dataclass(frozen=True)
class Element:
    name: str
    color: str
    node: tuple[int, int]
    kind: str  # 'trash' o 'deposit'

ELEMENTS = (
    Element('trash_V1_verde',   'verde',   (8, 12), 'trash'),
    Element('trash_V2_verde',   'verde',   (14, 12), 'trash'),
    Element('trash_N1_naranja', 'naranja', (8, 27), 'trash'),
    Element('trash_N2_naranja', 'naranja', (14, 27), 'trash'),
    Element('trash_A1_azul',    'azul',    (8, 32), 'trash'),
    Element('trash_A2_azul',    'azul',    (14, 7), 'trash'),
    Element('deposit_verde',   'verde',   (3, 10), 'deposit'),
    Element('deposit_naranja', 'naranja', (19, 30), 'deposit'),
    Element('deposit_azul',    'azul',    (11, 9), 'deposit'),
)

