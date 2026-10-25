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

import math

PKG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "src", "swarm_worlds")
OUT = os.path.join(PKG, "worlds", "pallet_warehouse.sdf")
MODEL = os.path.join(PKG, "models", "pallet_warehouse")

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

# Todos los visuales se FUNDEN en una sola malla COLLADA con un material por color
# (~10 draw calls): con ~1300 visuales sueltos cada camara pagaba un draw call
# por pieza y el RTF bajaba a 0.76 con 3 robots.
mesh = {}  # rgb -> [quads/tris en frame mundo]
col = []   # colisiones (primitivas)


def _add(rgb, faces):
    mesh.setdefault(rgb, []).extend(faces)


def box(name, x, y, z, sx, sy, sz, rgb, collide=False, yaw=0.0):
    c, s_ = math.cos(yaw), math.sin(yaw)
    hx, hy, hz = sx / 2, sy / 2, sz / 2

    def P(a, b, h):
        return (x + c * a - s_ * b, y + s_ * a + c * b, z + h)
    v = [P(a, b, h) for h in (-hz, hz) for b in (-hy, hy) for a in (-hx, hx)]
    # caras CCW vistas desde fuera
    _add(rgb, [(v[0], v[2], v[3], v[1]), (v[4], v[5], v[7], v[6]),
               (v[0], v[1], v[5], v[4]), (v[2], v[6], v[7], v[3]),
               (v[0], v[4], v[6], v[2]), (v[1], v[3], v[7], v[5])])
    if collide:
        col.append(f"""        <collision name="{name}_c"><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>""")


def cyl(name, x, y, z, r, h, rgb, collide=False, n=24):
    ring = [(x + r * math.cos(2 * math.pi * k / n), y + r * math.sin(2 * math.pi * k / n))
            for k in range(n)]
    z0, z1 = z - h / 2, z + h / 2
    faces = []
    for k in range(n):
        (ax, ay), (bx, by) = ring[k], ring[(k + 1) % n]
        faces.append(((ax, ay, z0), (bx, by, z0), (bx, by, z1), (ax, ay, z1)))
        faces.append(((x, y, z1), (ax, ay, z1), (bx, by, z1)))
        faces.append(((x, y, z0), (bx, by, z0), (ax, ay, z0)))
    _add(rgb, faces)
    if collide:
        col.append(f"""        <collision name="{name}_c"><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose><geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry></collision>""")


def write_mesh():
    """COLLADA con una <geometry> y un material por color (el .mtl de un OBJ no
    lo aplica Gazebo: salia todo blanco). Normales planas por cara."""
    os.makedirs(os.path.join(MODEL, "meshes"), exist_ok=True)
    lib_fx, lib_mat, lib_geo, nodes = [], [], [], []
    nv = 0
    for i, (rgb, faces) in enumerate(mesh.items()):
        pos, nrm, idx = [], [], []
        for face in faces:
            p0, p1, p2 = face[0], face[1], face[2]
            u = [p1[k] - p0[k] for k in range(3)]
            w = [p2[k] - p0[k] for k in range(3)]
            n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
            ln = math.sqrt(sum(c * c for c in n)) or 1.0
            ni = len(nrm) // 3
            nrm += [c / ln for c in n]
            base = len(pos) // 3
            for p in face:
                pos += list(p)
            for k in range(1, len(face) - 1):
                idx += [base, ni, base + k, ni, base + k + 1, ni]
        nv += len(pos) // 3
        g = f"g{i}"
        lib_fx.append(f'<effect id="fx{i}"><profile_COMMON><technique sid="c"><lambert>'
                      f'<ambient><color>{rgb} 1</color></ambient>'
                      f'<diffuse><color>{rgb} 1</color></diffuse></lambert></technique>'
                      f'</profile_COMMON></effect>')
        lib_mat.append(f'<material id="mat{i}"><instance_effect url="#fx{i}"/></material>')
        lib_geo.append(
            f'<geometry id="{g}"><mesh>'
            f'<source id="{g}p"><float_array id="{g}pa" count="{len(pos)}">'
            + " ".join(f"{c:.4f}" for c in pos) +
            f'</float_array><technique_common><accessor source="#{g}pa" count="{len(pos) // 3}" stride="3">'
            '<param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>'
            f'</accessor></technique_common></source>'
            f'<source id="{g}n"><float_array id="{g}na" count="{len(nrm)}">'
            + " ".join(f"{c:.4f}" for c in nrm) +
            f'</float_array><technique_common><accessor source="#{g}na" count="{len(nrm) // 3}" stride="3">'
            '<param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>'
            f'</accessor></technique_common></source>'
            f'<vertices id="{g}v"><input semantic="POSITION" source="#{g}p"/></vertices>'
            f'<triangles material="m{i}" count="{len(idx) // 6}">'
            f'<input semantic="VERTEX" source="#{g}v" offset="0"/>'
            f'<input semantic="NORMAL" source="#{g}n" offset="1"/>'
            f'<p>{" ".join(map(str, idx))}</p></triangles></mesh></geometry>')
        nodes.append(f'<node id="n{i}" name="n{i}"><instance_geometry url="#{g}"><bind_material>'
                     f'<technique_common><instance_material symbol="m{i}" target="#mat{i}"/>'
                     f'</technique_common></bind_material></instance_geometry></node>')
    dae = ('<?xml version="1.0" encoding="utf-8"?>\n'
           '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">\n'
           '<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>\n'
           f'<library_effects>{"".join(lib_fx)}</library_effects>\n'
           f'<library_materials>{"".join(lib_mat)}</library_materials>\n'
           f'<library_geometries>{chr(10).join(lib_geo)}</library_geometries>\n'
           f'<library_visual_scenes><visual_scene id="s">{"".join(nodes)}</visual_scene>'
           '</library_visual_scenes>\n<scene><instance_visual_scene url="#s"/></scene>\n</COLLADA>\n')
    with open(os.path.join(MODEL, "meshes", "pallet_warehouse.dae"), "w") as f:
        f.write(dae)
    return nv


def pallet_load(name, x, y, z, rng, sx=1.0, sy=1.05):
    box(f"{name}_pal", x, y, z + 0.075, sx, sy, 0.15, PALLET)
    h = rng.uniform(0.7, 1.15)
    box(f"{name}_load", x, y, z + 0.15 + h / 2, sx - 0.05, sy - 0.05, h, rng.choice(LOADS))


def rack_row(name, x0, x1, yc, rng):
    """Fila doble espalda con espalda centrada en yc, de x0 a x1."""
    col.append(f"""        <collision name="{name}_c"><pose>{(x0 + x1) / 2:.3f} {yc:.3f} {RACK_H / 2:.3f} 0 0 0</pose><geometry><box><size>{x1 - x0:.3f} {2 * ROW_D:.3f} {RACK_H:.3f}</size></box></geometry></collision>""")
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
    for n, x, y, sx, sy in (("n", 0, HY + WALL_T / 2, 2 * HX + 2 * WALL_T, WALL_T),
                            ("s", 0, -HY - WALL_T / 2, 2 * HX + 2 * WALL_T, WALL_T),
                            ("e", HX + WALL_T / 2, 0, WALL_T, 2 * HY),
                            ("w", -HX - WALL_T / 2, 0, WALL_T, 2 * HY)):
        box(f"wall_{n}", x, y, WALL_H / 2, sx, sy, WALL_H, WALL, collide=True)
        # zocalo oscuro hacia dentro
        ox = -0.01 if n == "e" else (0.01 if n == "w" else 0)
        oy = -0.01 if n == "n" else (0.01 if n == "s" else 0)
        box(f"band_{n}", x + ox, y + oy, 0.6, sx, sy, 1.2, WALL_BAND)

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
    col.append(f"""        <collision name="wrap_c"><pose>{ex + 0.2:.3f} {ey:.3f} 0.8 0 0 0</pose><geometry><box><size>2.8 2.3 1.6</size></box></geometry></collision>""")

    # Pilas de palets vacios junto a las paredes este/oeste de la zona central.
    for i, (x, y) in enumerate(((-13.6, -4.5), (-13.6, -2.8), (13.6, 2.5), (13.6, 4.2))):
        n = rng.randint(5, 9)
        h = 0.15 * n
        for k in range(n):
            box(f"stack{i}_{k}", x, y, 0.075 + 0.15 * k, 1.05, 1.0, 0.14, PALLET)
        col.append(f"""        <collision name="stack{i}_c"><pose>{x:.3f} {y:.3f} {h / 2:.3f} 0 0 0</pose><geometry><box><size>1.05 1.0 {h:.3f}</size></box></geometry></collision>""")

    # Columnas de la nave (con colision) en las 4 esquinas de la zona de paletizado.
    for i, (x, y) in enumerate(((-6.5, 6.5), (6.5, 6.5), (-6.5, -6.5), (6.5, -6.5))):
        box(f"pillar{i}", x, y, 2.0, 0.4, 0.4, 4.0, "0.55 0.55 0.58", collide=True)
        box(f"pillar{i}_g", x, y, 0.6, 0.44, 0.44, 1.2, ACCENT)

    sdf = f"""<?xml version="1.0"?>
<!--
  pallet_warehouse.sdf  -  almacen de paletizado con PASILLOS (generado).
  NO editar a mano: regenerar con scripts/gen_pallet_warehouse.py y despues
  /usr/bin/python3 scripts/world_maps.py - -worlds pallet_warehouse - -write
  (el mapa maps/pallet_warehouse.pgm es el que usa el launch para no colocar
  robots/basura/depositos dentro de las estanterias).
  Nave 30x50 m, sin techo (cenital a 38 m como tugbot_warehouse). Spawn:
  center 0,0 y area_half 24 (el mapa recorta todo lo que no es pasillo).
  Visual = UNA malla COLLADA (models/pallet_warehouse/meshes, un material por color);
  colision = primitivas.
-->
<sdf version="1.9">
  <world name="world_demo">
    <gravity>0 0 -9.8</gravity>
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <max_contacts>10</max_contacts>
    </physics>
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system"/>
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="gz::sim::systems::Imu" filename="gz-sim-imu-system"/>
    <plugin name="gz::sim::systems::Sensors" filename="gz-sim-sensors-system">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.62 0.72 0.80 1</background>
      <shadows>0</shadows>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>0</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.95 0.95 0.92 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -1</direction>
    </light>

    <model name="pallet_warehouse">
      <static>true</static>
      <link name="link">
{chr(10).join(col)}
        <visual name="visual">
          <geometry><mesh><uri>model://pallet_warehouse/meshes/pallet_warehouse.dae</uri></mesh></geometry>
        </visual>
      </link>
    </model>

    <model name="overhead_camera">
      <static>true</static>
      <pose>0 0 38 0 1.5707963 0</pose>
      <link name="link">
        <sensor name="overhead" type="camera">
          <topic>/overhead/image</topic>
          <update_rate>15</update_rate>
          <camera>
            <horizontal_fov>1.3</horizontal_fov>
            <image><width>900</width><height>900</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>80.0</far></clip>
          </camera>
          <always_on>1</always_on>
          <visualize>false</visualize>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
"""
    nv = write_mesh()
    with open(os.path.join(MODEL, "model.config"), "w") as f:
        f.write('<?xml version="1.0"?>\n<model><name>pallet_warehouse</name><version>1.0</version>'
                '<sdf version="1.9">model.sdf</sdf><description>Malla visual de '
                'worlds/pallet_warehouse.sdf (generada)</description></model>\n')
    with open(OUT, "w") as f:
        f.write(sdf)
    print(f"{OUT}: {sum(map(len, mesh.values()))} caras ({nv} vertices) en "
          f"{len(mesh)} materiales, {len(col)} colisiones")


if __name__ == "__main__":
    main()
