#!/usr/bin/env python3
"""Genera worlds/pallet_warehouse.sdf: almacen de paletizado con PASILLOS.

Nave de 30x50 m (mismo tamano que tugbot_warehouse, misma cenital) con:
  - 2 bloques de estanterias de palets al norte y 2 al sur (filas dobles
    espalda con espalda, pasillos de 3.4 m entre filas),
  - un pasillo central de 4 m (x en [-2, 2]) que cruza toda la nave y
    pasillos laterales de 3 m contra las paredes,
  - una zona de paletizado en el centro (y en [-6.5, 6.5]) con una linea de
    transportador, una enfardadora y pilas de palets vacios.

Paleta pensada para el dataset ACT (CONTRASTE): suelo gris OSCURO para que
resalten el robot (gris claro desde arriba) y la basura; el decorado NO usa
verde, azul ni naranja (los colores de basura/depositos) para que la politica
no confunda una estanteria con un deposito. Montantes rojo oscuro, cargas
blancas/negras/moradas, palets de plastico gris claro.

Colision solo con cajas/cilindros (una caja por fila doble de estanteria) y SIN
ground_plane: fuera de la nave no hay suelo -> world_maps.py lo marca ocupado y
el launch nunca coloca nada fuera.

    /usr/bin/python3 scripts/gen_pallet_warehouse.py
    /usr/bin/python3 scripts/world_maps.py --worlds pallet_warehouse --write
"""
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from world_builder import Builder  # noqa: E402

HX, HY = 15.0, 25.0          # semilados interiores de la nave
WALL_T, WALL_H = 0.3, 4.0

# Colores (r g b). Nada de verde/azul/naranja (reservados a basura/depositos).
FLOOR = "0.17 0.17 0.19"
WALL = "0.86 0.84 0.80"
WALL_BAND = "0.30 0.30 0.33"
UPRIGHT = "0.62 0.08 0.12"
BEAM = "0.60 0.62 0.65"
PALLET = "0.78 0.78 0.80"
LOADS = ["0.92 0.92 0.94", "0.92 0.92 0.94", "0.15 0.15 0.17", "0.48 0.26 0.58"]
LINE = "0.95 0.95 0.95"
MACHINE = "0.30 0.30 0.33"
ACCENT = "0.95 0.82 0.10"    # amarillo de seguridad, solo en maquinas

# Estanterias: bloques en x, filas dobles (2 x 1.1 m) en y.
BLOCKS_X = [(-12.0, -2.0), (2.0, 12.0)]   # pasillo central 4 m, laterales 3 m
ROWS_Y = [9.6, 15.2, 20.8]   # centros de las filas dobles norte (sur = espejo)
ROW_D = 1.1                  # fondo de una fila simple
BAY = 2.5
LEVELS = [0.0, 1.5, 3.0]     # base de cada nivel de carga
RACK_H = 4.4

B = Builder("pallet_warehouse")
box, cyl = B.box, B.cyl


def pallet_load(name, x, y, z, rng, sx=1.0, sy=1.05):
    box(f"{name}_pal", x, y, z + 0.075, sx, sy, 0.15, PALLET)
    h = rng.uniform(0.7, 1.15)
    box(f"{name}_load", x, y, z + 0.15 + h / 2, sx - 0.05, sy - 0.05, h, rng.choice(LOADS))


def rack_row(name, x0, x1, yc, rng):
    """Fila doble espalda con espalda centrada en yc, de x0 a x1."""
    B.collision_box(name, (x0 + x1) / 2, yc, RACK_H / 2, x1 - x0, 2 * ROW_D, RACK_H)
    nb = int(round((x1 - x0) / BAY))
    for side, ys in (("a", yc - ROW_D / 2), ("b", yc + ROW_D / 2)):
        yf = ys - ROW_D / 2 + 0.05 if side == "a" else ys + ROW_D / 2 - 0.05   # cara de pasillo
        yb = ys + ROW_D / 2 - 0.05 if side == "a" else ys - ROW_D / 2 + 0.05
        for k in range(nb + 1):
            x = x0 + k * BAY
            x = min(max(x, x0 + 0.05), x1 - 0.05)
            for j, yy in enumerate((yf, yb)):
                box(f"{name}{side}_up{k}_{j}", x, yy, RACK_H / 2, 0.09, 0.09, RACK_H, UPRIGHT)
        for li, z in enumerate(LEVELS[1:] + [RACK_H - 0.1]):
            for j, yy in enumerate((yf, yb)):
                box(f"{name}{side}_bm{li}_{j}", (x0 + x1) / 2, yy, z - 0.07,
                    x1 - x0, 0.06, 0.13, BEAM)
        for k in range(nb):
            for li, z in enumerate(LEVELS):
                for p in range(2):
                    if rng.random() < 0.75:
                        x = x0 + k * BAY + BAY * (0.27 + 0.46 * p)
                        pallet_load(f"{name}{side}_l{k}_{li}_{p}", x, ys, z + (0.0 if li == 0 else 0.01),
                                    rng, sx=1.0, sy=ROW_D - 0.1)


def main():
    rng = random.Random(7)

    # Suelo (colision + visual) y paredes con zocalo oscuro (referencia en la frontal).
    box("floor", 0, 0, -0.05, 2 * HX + 2 * WALL_T, 2 * HY + 2 * WALL_T, 0.1, FLOOR, collide=True)
    B.walls(HX, HY, WALL_T, WALL_H, WALL, WALL_BAND)

    # Muelles de carga: portones oscuros en las paredes este/oeste (landmarks).
    for i, y in enumerate((-17.0, -1.5, 1.5, 17.0)):
        box(f"dock_e{i}", HX - 0.01, y, 1.8, 0.04, 2.6, 3.0, MACHINE)
        box(f"dock_e{i}_s", HX - 0.03, y, 3.35, 0.04, 2.8, 0.12, ACCENT)
    for i, y in enumerate((-10.0, 10.0)):
        box(f"dock_w{i}", -HX + 0.01, y, 1.8, 0.04, 2.6, 3.0, MACHINE)
        box(f"dock_w{i}_s", -HX + 0.03, y, 3.35, 0.04, 2.8, 0.12, ACCENT)

    # Estanterias norte y sur.
    for bi, (x0, x1) in enumerate(BLOCKS_X):
        for ri, yc in enumerate(ROWS_Y):
            rack_row(f"rk_n{bi}{ri}", x0, x1, yc, rng)
            rack_row(f"rk_s{bi}{ri}", x0, x1, -yc, rng)

    # Lineas de suelo blancas: bordes del pasillo central y de la zona de paletizado.
    zl = 0.003
    for s in (-1, 1):
        box(f"ln_main{s}", s * 1.9, 0, zl, 0.10, 2 * HY - 0.4, 0.006, LINE)
        box(f"ln_zone{s}", 0, s * 6.5, zl, 2 * HX - 0.4, 0.10, 0.006, LINE)
    # cebreado de paso de peatones en las entradas del pasillo central
    for s in (-1, 1):
        for k in range(5):
            box(f"zebra{s}_{k}", -1.5 + k * 0.75, s * 7.4, zl, 0.35, 1.2, 0.006, LINE)

    # Zona de paletizado (centro). Transportador en la mitad oeste.
    cy = 4.2
    box("conv_body", -8.5, cy, 0.38, 8.0, 0.9, 0.76, MACHINE, collide=True)
    box("conv_belt", -8.5, cy, 0.77, 8.0, 0.8, 0.02, "0.08 0.08 0.08")
    box("conv_rail_a", -8.5, cy - 0.47, 0.85, 8.0, 0.05, 0.14, ACCENT)
    box("conv_rail_b", -8.5, cy + 0.47, 0.85, 8.0, 0.05, 0.14, ACCENT)
    for k in range(5):
        c = rng.choice(LOADS[1:])
        box(f"conv_box{k}", -11.5 + k * 1.5, cy, 0.98, 0.5, 0.4, 0.4, c)
    # celda del robot paletizador al final del transportador
    cyl("pal_robot_base", -3.9, cy, 0.45, 0.45, 0.9, MACHINE, collide=True)
    cyl("pal_robot_arm", -3.9, cy, 1.25, 0.18, 0.7, ACCENT)
    box("pal_out", -3.9, cy - 1.6, 0.65, 1.0, 1.05, 1.3, "0.92 0.92 0.94", collide=True)
    box("pal_out_pal", -3.9, cy - 1.6, 0.075, 1.0, 1.05, 0.15, PALLET)

    # Enfardadora (stretch wrapper) en la mitad este: mesa giratoria + mastil.
    ex, ey = 8.5, -3.5
    cyl("wrap_table", ex, ey, 0.05, 1.1, 0.1, MACHINE)
    box("wrap_load", ex, ey, 0.75, 1.0, 1.05, 1.3, "0.48 0.26 0.58")
    box("wrap_mast", ex + 1.45, ey, 1.25, 0.25, 0.25, 2.5, ACCENT)
    B.collision_box("wrap", ex + 0.2, ey, 0.8, 2.8, 2.3, 1.6)

    # Pilas de palets vacios junto a las paredes este/oeste de la zona central.
    for i, (x, y) in enumerate(((-13.6, -4.5), (-13.6, -2.8), (13.6, 2.5), (13.6, 4.2))):
        n = rng.randint(5, 9)
        h = 0.15 * n
        for k in range(n):
            box(f"stack{i}_{k}", x, y, 0.075 + 0.15 * k, 1.05, 1.0, 0.14, PALLET)
        B.collision_box(f"stack{i}", x, y, h / 2, 1.05, 1.0, h)

    # Columnas de la nave (con colision) en las 4 esquinas de la zona de paletizado.
    for i, (x, y) in enumerate(((-6.5, 6.5), (6.5, 6.5), (-6.5, -6.5), (6.5, -6.5))):
        box(f"pillar{i}", x, y, 2.0, 0.4, 0.4, 4.0, "0.55 0.55 0.58", collide=True)
        box(f"pillar{i}_g", x, y, 0.6, 0.44, 0.44, 1.2, ACCENT)

    B.write("""  pallet_warehouse.sdf  -  almacen de paletizado con PASILLOS (generado).
  NO editar a mano: regenerar con scripts/gen_pallet_warehouse.py y despues
  /usr/bin/python3 scripts/world_maps.py - -worlds pallet_warehouse - -write
  (el mapa maps/pallet_warehouse.pgm es el que usa el launch para no colocar
  robots/basura/depositos dentro de las estanterias).
  Nave 30x50 m, sin techo (cenital a 38 m como tugbot_warehouse). Spawn:
  center 0,0 y area_half 24 (el mapa recorta todo lo que no es pasillo).
  Visual = UNA malla COLLADA (models/pallet_warehouse/meshes, un material por color);
  colision = primitivas.""")

if __name__ == "__main__":
    main()
