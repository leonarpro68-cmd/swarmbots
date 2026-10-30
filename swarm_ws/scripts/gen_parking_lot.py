#!/usr/bin/env python3
"""Genera worlds/parking_lot.sdf: APARCAMIENTO exterior de 30x50 m.

  - 4 columnas de plazas en bateria (2.5 x 5 m): una contra cada muro y una
    doble en el centro, con ~65 % de plazas ocupadas por coches,
  - 2 calles de 4.5 m a lo largo (x en [-10,-5.5] y [5.5,10]) y una calle
    transversal de 5 m en y en [-2.5, 2.5] que las une,
  - farolas en la isleta central, muro perimetral bajo de hormigon y una
    caseta de cobro.

Paleta (contraste para el ACT): asfalto casi negro con lineas blancas; coches
rojo/blanco/negro/morado/amarillo/plata. Nada de verde/azul/naranja.

    /usr/bin/python3 scripts/gen_parking_lot.py
    /usr/bin/python3 scripts/world_maps.py --worlds parking_lot --write
"""
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from world_builder import Builder  # noqa: E402

HX, HY = 15.0, 25.0
ASPHALT = "0.13 0.13 0.14"
SIDEWALK = "0.62 0.60 0.56"
LINE = "0.95 0.95 0.95"
YELLOW = "0.95 0.82 0.10"
CONCRETE = "0.78 0.77 0.74"
POLE = "0.35 0.35 0.38"
TIRE = "0.06 0.06 0.06"
GLASS = "0.20 0.22 0.26"
CARS = ["0.75 0.10 0.12", "0.93 0.93 0.95", "0.10 0.10 0.11", "0.45 0.24 0.58",
        "0.92 0.78 0.12", "0.68 0.69 0.72", "0.93 0.93 0.95", "0.68 0.69 0.72"]

STALL_W, STALL_D = 2.5, 5.0
# columnas de plazas: (x del fondo de la plaza, sentido hacia la calle)
COLUMNS = [(-HX, +1), (-0.25, -1), (0.25, +1), (HX, -1)]
CROSS = 2.5   # semiancho de la calle transversal

B = Builder("parking_lot")
box, cyl = B.box, B.cyl


def car(name, x, y, along_x, rgb):
    """Coche de 4.4 x 1.8 m; along_x = eje largo en x. Colision hasta el suelo
    (que el robot no intente meterse por debajo)."""
    L, W = (4.4, 1.8) if along_x else (1.8, 4.4)
    cL, cW = (2.3, 1.6) if along_x else (1.6, 2.3)
    box(f"{name}_body", x, y, 0.6, L, W, 0.7, rgb)
    box(f"{name}_cab", x - (0.2 if along_x else 0), y, 1.22, cL, cW, 0.55, GLASS)
    box(f"{name}_roof", x - (0.2 if along_x else 0), y, 1.5, cL - 0.2, cW - 0.1, 0.04, rgb)
    for i, (a, b) in enumerate(((1.4, 0.8), (1.4, -0.8), (-1.4, 0.8), (-1.4, -0.8))):
        wx, wy = (a, b) if along_x else (b, a)
        box(f"{name}_w{i}", x + wx, y + wy, 0.32, 0.64 if along_x else 0.24,
            0.24 if along_x else 0.64, 0.64, TIRE)
    B.collision_box(name, x, y, 0.76, L, W, 1.52)


def main():
    rng = random.Random(11)

    # Suelo de asfalto (colision + visual) y acera exterior solo visual, por
    # debajo (fuera del muro no hay suelo de colision -> el mapa lo bloquea).
    box("floor", 0, 0, -0.05, 2 * HX + 0.6, 2 * HY + 0.6, 0.1, ASPHALT, collide=True)
    box("sidewalk", 0, 0, -0.11, 2 * HX + 12, 2 * HY + 12, 0.1, SIDEWALK)
    B.walls(HX, HY, 0.3, 1.0, CONCRETE)

    # Plazas: lineas separadoras + coches.
    zl = 0.003
    n_car = 0
    for ci, (xb, d) in enumerate(COLUMNS):
        xc = xb + d * STALL_D / 2          # centro de la plaza en x
        for side in (-1, 1):               # mitad sur / norte de la calle transversal
            y0 = side * (CROSS + 0.5)
            k = 0
            while True:
                ya = y0 + side * k * STALL_W
                yb = ya + side * STALL_W
                if abs(yb) > HY - 0.3:
                    break
                box(f"ln{ci}_{side}_{k}", xc, ya, zl, STALL_D, 0.10, 0.006, LINE)
                if rng.random() < 0.65:
                    car(f"car{n_car}", xc - d * 0.2, (ya + yb) / 2, True, rng.choice(CARS))
                    n_car += 1
                k += 1
            box(f"ln{ci}_{side}_{k}", xc, ya, zl, STALL_D, 0.10, 0.006, LINE)
        # linea de fondo hacia la calle
        box(f"lnf{ci}", xb + d * STALL_D, 0, zl, 0.10, 2 * HY - 0.6, 0.006, LINE)

    # Isleta central con farolas (entre las dos columnas centrales).
    # (cortada en la calle transversal: si no, parte el aparcamiento en dos)
    ly = (HY - 0.3 - CROSS) / 2
    for s in (-1, 1):
        box(f"island{s}", 0, s * (CROSS + ly), 0.08, 0.5, 2 * ly, 0.16, CONCRETE, collide=True)
    for i, y in enumerate((-20, -10, 10, 20)):
        cyl(f"lamp{i}", 0, y, 3.0, 0.1, 6.0, POLE, collide=True)
        box(f"lamp{i}_head", 0, y, 6.0, 1.2, 0.3, 0.12, POLE)

    # Calle transversal: cebra amarilla en el cruce de cada calle.
    for s in (-1, 1):
        xm = s * 7.75
        for k in range(6):
            box(f"zebra{s}_{k}", xm - 1.9 + k * 0.75, 0, zl, 0.35, 2 * CROSS - 1.0, 0.006, YELLOW)
    # flechas de sentido (una por calle y mitad)
    for s in (-1, 1):
        for t in (-1, 1):
            x, y = s * 7.75, t * 14
            box(f"arrow{s}{t}", x, y, zl, 0.25, 2.0, 0.006, LINE)
            box(f"arrowh{s}{t}", x, y + t * 1.1, zl, 0.9, 0.25, 0.006, LINE)

    # Caseta de cobro en la entrada sur de la calle oeste, con barrera.
    box("booth", -12.2, -22.5, 1.25, 1.6, 1.6, 2.5, CONCRETE, collide=True)
    box("booth_roof", -12.2, -22.5, 2.6, 2.0, 2.0, 0.2, "0.75 0.10 0.12")
    box("barrier", -9.0, -21.0, 1.0, 4.0, 0.08, 0.08, "0.75 0.10 0.12")

    B.write("""  parking_lot.sdf  -  APARCAMIENTO exterior 30x50 m (generado).
  NO editar a mano: regenerar con scripts/gen_parking_lot.py y despues
  /usr/bin/python3 scripts/world_maps.py - -worlds parking_lot - -write.
  Spawn: center 0,0 y area_half 24 (el mapa deja solo calles y plazas libres).
  Visual = UNA malla COLLADA (models/parking_lot/meshes); colision = primitivas.""",
            ambient="0.65 0.65 0.65", background="0.55 0.70 0.88",
            sun_diffuse="1.0 0.97 0.90", sun_dir="-0.4 -0.3 -1")
    print(f"{n_car} coches")


if __name__ == "__main__":
    main()
