#!/usr/bin/env python3
"""Da una paleta PROPIA a rubber_factory y electrical_substation.

Tras el recorte (crop_world.py) los dos mundos eran gris sobre gris y se
confundian entre si. Este script (idempotente; correrlo DESPUES de
crop_world.py, que regenera los modelos *_crop) los pinta:

  rubber_factory: suelo de epoxi crema, cada linea de maquinas de un color,
    pared interior de ladrillo, objetos bajos negros, perimetro pizarra.
    La malla visual unica se parte en un STL por componente conexa.
  electrical_substation: patio de asfalto, losa de hormigon, edificios de
    ladrillo, casa amarilla, equipos galvanizados, valla oscura.

Regla de color del dataset ACT: NADA de verde/azul/naranja en el decorado
(son los colores de basura y depositos).

    /usr/bin/python3 scripts/paint_worlds.py
"""
import os
import re
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import world_geometry as wg  # noqa: E402

PKG = os.path.join(os.path.dirname(HERE), "src", "swarm_worlds")

RED = "0.72 0.12 0.14"
YELLOW = "0.92 0.76 0.12"
PURPLE = "0.46 0.26 0.62"
WHITE = "0.93 0.93 0.95"
BLACK = "0.16 0.16 0.18"
BRICK = "0.58 0.22 0.18"
SLATE = "0.28 0.28 0.32"
CREAM = "0.86 0.82 0.70"


def mat(rgb):
    return (f"<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse>"
            f"<specular>0.05 0.05 0.05 1</specular></material>")


def sub_material(xml, start_tag, rgb):
    """Sustituye el <material>...</material> que sigue a start_tag."""
    i = xml.index(start_tag)
    a = xml.index("<material>", i)
    b = xml.index("</material>", a) + len("</material>")
    return xml[:a] + mat(rgb) + xml[b:]


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"paint_worlds.py".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack("<12fH", *n, *t[0], *t[1], *t[2], 0))


def components(tris):
    v, inv = np.unique(np.round(tris.reshape(-1, 3), 3), axis=0, return_inverse=True)
    idx = inv.reshape(-1, 3)
    par = np.arange(len(v))

    def f(a):
        while par[a] != a:
            par[a] = par[par[a]]
            a = par[a]
        return a
    for a, b, c in idx:
        ra = f(a)
        par[f(b)] = ra
        par[f(c)] = ra
    comp = np.array([f(i[0]) for i in idx])
    return [tris[comp == c] for c in np.unique(comp)]


def paint_factory():
    mdir = os.path.join(PKG, "models", "rubber_factory_crop")
    src = os.path.join(mdir, "meshes", "emka_factory__link_1__visual__vis.stl")
    parts = sorted(components(wg.load_stl(src)), key=lambda t: t.reshape(-1, 3)[:, 0].min())
    machines = [RED, YELLOW, PURPLE, WHITE, YELLOW, RED]
    groups = {}
    for t in parts:
        p = t.reshape(-1, 3)
        size = p.max(0) - p.min(0)
        if size[1] > 45:                      # pared interior a lo largo de toda la nave
            key = ("wall", BRICK)
        elif size[2] > 1.5:                   # lineas de maquinas (2 m de alto)
            k = len([g for g in groups if g[0].startswith("machine")])
            key = (f"machine{k}", machines[k % len(machines)])
        else:                                 # objetos bajos
            key = ("small", BLACK)
        groups.setdefault(key, []).append(t)
    vis = []
    for (name, rgb), ts in groups.items():
        fn = f"emka_factory__{name}__vis.stl"
        write_stl(os.path.join(mdir, "meshes", fn), np.concatenate(ts))
        vis.append(f"""      <visual name="emka_factory__{name}">
        <geometry><mesh><uri>model://rubber_factory_crop/meshes/{fn}</uri></mesh></geometry>
        {mat(rgb)}
      </visual>""")
    p = os.path.join(mdir, "model.sdf")
    xml = open(p).read()
    xml = re.sub(r'\s*<visual name="emka_factory__[^"]*">.*?</visual>', "", xml, flags=re.S)
    xml = xml.replace("    </link>", "\n".join(vis) + "\n    </link>", 1)
    open(p, "w").write(xml)

    p = os.path.join(PKG, "worlds", "rubber_factory.sdf")
    xml = open(p).read()
    xml = sub_material(xml, "<model name='ground_plane'>", CREAM)
    xml = re.sub(r"(<model name=\"perimeter_walls\">.*?</model>)",
                 lambda m: re.sub(r"<material>.*?</material>", mat(SLATE), m.group(1), flags=re.S),
                 xml, flags=re.S)
    xml = xml.replace("<background>0.7 0.7 0.7 1</background>", "<background>0.80 0.84 0.88 1</background>")
    open(p, "w").write(xml)
    print(f"rubber_factory: {len(groups)} grupos de color")


def paint_substation():
    p = os.path.join(PKG, "models", "electrical_substation_crop", "model.sdf")
    xml = open(p).read()
    for name, rgb in (("station_base__visual", "0.45 0.44 0.42"),       # losa de hormigon
                      ("station_buildings__visual", BRICK),
                      ("station_house__visual", "0.95 0.88 0.55"),      # amarillo claro
                      ("station_robothouse__visual", WHITE),
                      ("station_fence__visual", SLATE),
                      ("station_electrical_stuff_part_1__visual", "0.74 0.76 0.80"),  # galvanizado
                      ("summit_docking_station__base_visual", YELLOW)):
        xml = sub_material(xml, f'<visual name="{name}">', rgb)
    open(p, "w").write(xml)

    p = os.path.join(PKG, "worlds", "electrical_substation.sdf")
    xml = open(p).read()
    xml = sub_material(xml, '<model name="ground_plane">', "0.26 0.26 0.28")   # patio de asfalto
    xml = re.sub(r"(<model name=\"perimeter_walls\">.*?</model>)",
                 lambda m: re.sub(r"<material>.*?</material>", mat("0.80 0.79 0.76"), m.group(1), flags=re.S),
                 xml, flags=re.S)
    open(p, "w").write(xml)
    print("electrical_substation: pintado")


if __name__ == "__main__":
    paint_factory()
    paint_substation()
