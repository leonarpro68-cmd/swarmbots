#!/usr/bin/env python3
"""Genera worlds/supermarket.sdf: SUPERMERCADO de 30x50 m con GENTE CAMINANDO.

  - 5 gondolas dobles (1 x 16 m, 1.8 m de alto) con pasillos de 4 m, llenas
    de producto de colores; cabeceras rojas,
  - neveras a lo largo de la pared norte e islas de fruta delante,
  - 4 cajas de cobro y la entrada al sur,
  - suelo en damero crema / gris (los robots y la basura contrastan con
    ambos tonos),
  - 5 personas (<actor>) que recorren el pasillo central, dos pasillos de
    gondolas, la zona de cajas y la de fruta en bucle.

Las personas NO tienen colision fisica en Harmonic (los robots las
atravesarian): el LiDAR y las camaras SI las ven. world_maps.py marca su
recorrido como ocupado (+-1 m), asi que nunca se coloca basura ni robots encima.

Animacion: models/person_{red,white}/meshes/walk.dae (actor de Fuel
mingfei/actor RECOLOREADO: venia con jersey verde y vaqueros azules).

Paleta: producto rojo/amarillo/morado/blanco/rosa/marron; nada de
verde/azul/naranja (colores de basura y depositos).

    /usr/bin/python3 scripts/gen_supermarket.py
    /usr/bin/python3 scripts/world_maps.py --worlds supermarket --write
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from world_builder import Builder  # noqa: E402

HX, HY = 15.0, 25.0
TILE_A = "0.86 0.84 0.78"
TILE_B = "0.55 0.55 0.57"
WALL = "0.94 0.93 0.90"
WALL_BAND = "0.62 0.10 0.14"
SHELF = "0.80 0.80 0.82"
ENDCAP = "0.75 0.10 0.12"
PRODUCTS = ["0.80 0.12 0.15", "0.95 0.82 0.15", "0.50 0.28 0.62", "0.95 0.95 0.95",
            "0.90 0.45 0.65", "0.45 0.30 0.22", "0.20 0.20 0.22"]
FRIDGE = "0.92 0.93 0.95"
GLASS = "0.30 0.32 0.36"
COUNTER = "0.30 0.30 0.33"
WALK_SPEED = 0.9   # m/s (la animacion walk.dae avanza ~1 m/s)

GONDOLAS_X = [-10.0, -5.0, 0.0, 5.0, 10.0]
G_Y0, G_Y1 = -4.0, 12.0
G_D, G_H = 1.0, 1.8

B = Builder("supermarket")
box, cyl = B.box, B.cyl


def gondola(name, x, rng):
    yc, L = (G_Y0 + G_Y1) / 2, G_Y1 - G_Y0
    B.collision_box(name, x, yc, G_H / 2, G_D, L, G_H)
    box(f"{name}_back", x, yc, G_H / 2, 0.06, L, G_H, SHELF)
    box(f"{name}_base", x, yc, 0.06, G_D, L, 0.12, SHELF)
    for k, z in enumerate((0.55, 1.0, 1.45)):
        box(f"{name}_sh{k}", x, yc, z, G_D, L, 0.03, SHELF)
    # producto: bloques de color por tramo de 1 m, a cada lado y en cada balda
    for side in (-1, 1):
        for k, z in enumerate((0.12, 0.57, 1.02, 1.47)):
            y = G_Y0 + 0.1
            while y < G_Y1 - 0.4:
                w = rng.uniform(0.6, 1.4)
                w = min(w, G_Y1 - 0.1 - y)
                h = rng.uniform(0.22, 0.36)
                box(f"{name}_p{side}{k}_{y:.1f}", x + side * 0.27, y + w / 2, z + h / 2,
                    0.4, w - 0.04, h, rng.choice(PRODUCTS))
                y += w
    for s, yy in ((-1, G_Y0 - 0.15), (1, G_Y1 + 0.15)):
        box(f"{name}_cap{s}", x, yy, G_H / 2 + 0.1, G_D + 0.1, 0.3, G_H + 0.2, ENDCAP)
    B.collision_box(f"{name}_caps", x, (G_Y0 + G_Y1) / 2, G_H / 2, G_D + 0.1, L + 0.6, G_H)


def actor(name, model, pts, z=1.0):
    """Recorrido cerrado a WALK_SPEED; en cada esquina gira en 0.6 s sin avanzar."""
    wps, t = [], 0.0
    n = len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        yaw = math.atan2(y1 - y0, x1 - x0)
        wps.append((t, x0, y0, yaw))
        t += 0.6
        wps.append((t, x0, y0, yaw))
        t += math.hypot(x1 - x0, y1 - y0) / WALK_SPEED
        wps.append((t, x1, y1, yaw))
        t += 0.01
    wx = "\n".join(f"          <waypoint><time>{tt:.2f}</time><pose>{x:.2f} {y:.2f} {z} 0 0 {yw:.3f}</pose></waypoint>"
                   for tt, x, y, yw in wps)
    return f"""
    <actor name="{name}">
      <skin><filename>model://{model}/meshes/walk.dae</filename><scale>1.0</scale></skin>
      <animation name="walking">
        <filename>model://{model}/meshes/walk.dae</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
{wx}
        </trajectory>
      </script>
    </actor>
"""


def main():
    rng = random.Random(5)

    # Suelo: caja de colision + damero visual de 2 m.
    B.collision_box("floor", 0, 0, -0.05, 2 * HX + 0.6, 2 * HY + 0.6, 0.1)
    for i in range(int(2 * HX / 2)):
        for j in range(int(2 * HY / 2)):
            x, y = -HX + 1 + 2 * i, -HY + 1 + 2 * j
            box(f"tile{i}_{j}", x, y, -0.05, 2.0, 2.0, 0.1, TILE_A if (i + j) % 2 else TILE_B)
    B.walls(HX, HY, 0.3, 3.5, WALL, WALL_BAND, band_h=0.4)

    for i, x in enumerate(GONDOLAS_X):
        gondola(f"gon{i}", x, rng)

    # Neveras contra la pared norte (dejando hueco en las esquinas).
    for i, x in enumerate(range(-12, 13, 3)):
        box(f"fridge{i}", x, HY - 0.5, 1.0, 2.9, 1.0, 2.0, FRIDGE)
        box(f"fridge{i}_g", x, HY - 1.01, 1.1, 2.6, 0.02, 1.5, GLASS)
    B.collision_box("fridges", 0, HY - 0.5, 1.0, 26.9, 1.0, 2.0)

    # Islas de fruta (mesas bajas con cajas de colores) entre neveras y gondolas.
    for i, x in enumerate((-8.0, 0.0, 8.0)):
        y = 18.5
        box(f"isl{i}", x, y, 0.45, 3.0, 1.6, 0.9, "0.45 0.30 0.22", collide=True)
        for a in range(3):
            for b in range(2):
                box(f"isl{i}_f{a}{b}", x - 1.0 + a, y - 0.4 + 0.8 * b, 0.97, 0.9, 0.7, 0.14,
                    rng.choice(["0.80 0.12 0.15", "0.95 0.82 0.15", "0.50 0.28 0.62", "0.95 0.95 0.95"]))

    # Cajas de cobro (cinta + puesto) y entrada.
    for i, x in enumerate((-9.0, -4.5, 4.5, 9.0)):
        y = -13.0
        box(f"chk{i}", x, y, 0.45, 0.8, 3.0, 0.9, COUNTER, collide=True)
        box(f"chk{i}_belt", x, y + 0.3, 0.91, 0.6, 2.2, 0.02, "0.08 0.08 0.08")
        box(f"chk{i}_reg", x, y - 1.1, 1.15, 0.4, 0.4, 0.5, "0.95 0.82 0.15")
        cyl(f"chk{i}_light", x, y - 1.3, 1.8, 0.05, 1.8, ENDCAP)
    box("entrance", 0, -HY + 0.01, 1.2, 6.0, 0.06, 2.4, GLASS)
    box("entrance_frame", 0, -HY + 0.02, 2.5, 6.4, 0.08, 0.2, ENDCAP)
    # Carros apilados junto a la entrada.
    for i in range(4):
        box(f"cart{i}", 11.0, -22.0 + 0.5 * i, 0.5, 0.6, 0.9, 1.0, "0.70 0.70 0.74")
    B.collision_box("carts", 11.0, -21.25, 0.5, 0.6, 2.4, 1.0)

    # Gente: rutas cerradas por zonas de paso (no por todos los pasillos: lo que
    # recorre un actor queda vetado para colocar basura).
    people = [
        ("person0", "person_red",   [(-2.5, -8.0), (-2.5, 15.0)]),                       # pasillo central
        ("person1", "person_white", [(-7.5, -2.0), (-7.5, 10.0)]),                       # pasillo de gondolas
        ("person2", "person_red",   [(-12.0, -8.5), (12.0, -8.5)]),                      # delante de cajas
        ("person3", "person_white", [(-11.5, 15.5), (11.5, 15.5), (11.5, 21.5), (-11.5, 21.5)]),  # fruta
        ("person4", "person_red",   [(7.5, 10.0), (7.5, -2.0)]),                         # pasillo de gondolas
    ]
    extra = "".join(actor(n, m, p) for n, m, p in people)

    B.write("""  supermarket.sdf  -  SUPERMERCADO 30x50 m con 5 personas caminando (generado).
  NO editar a mano: regenerar con scripts/gen_supermarket.py y despues
  /usr/bin/python3 scripts/world_maps.py - -worlds supermarket - -write.
  Spawn: center 0,0 y area_half 24. Las personas son <actor>: sin colision
  fisica, visibles al LiDAR y a las camaras; su recorrido esta vetado en el mapa.
  Visual = UNA malla COLLADA (models/supermarket/meshes); colision = primitivas.""",
            ambient="0.7 0.7 0.7", background="0.85 0.86 0.88",
            sun_diffuse="0.9 0.9 0.9", sun_dir="-0.2 0.3 -1", extra=extra)


if __name__ == "__main__":
    main()
