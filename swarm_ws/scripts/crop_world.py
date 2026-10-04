#!/usr/bin/env python3
"""Recorta un mundo SDF grande a una ventana rectangular (p.ej. 30 x 50 m, el
tamano de tugbot_warehouse) y lo deja centrado en el origen y cerrado con
paredes perimetrales.

Que hace, con la geometria REAL del mundo (world_geometry.py, poses 6D):
  1. Cada <collision>/<visual> de malla o primitiva de los <include> se lleva a
     frame mundo, se traslada para que el centro de la ventana quede en (0,0)
     y se CORTA contra el rectangulo [-wx/2, wx/2] x [-wy/2, wy/2] (corte de
     Sutherland-Hodgman en XY, z interpolada). Lo que cae fuera desaparece.
  2. Los trozos se guardan como STL en models/<world>_crop/meshes/ y se juntan
     en UN modelo estatico models/<world>_crop/model.sdf. Material, sombras,
     transparencia (visual) y <surface> (colision) se copian LITERALES.
  3. Los <model> inline (ground_plane...) y las <light> se TRASLADAN igual,
     anadiendo/componiendo su <pose>: un ground_plane finito colocado para el
     mundo original (la fabrica lo tiene en (-30,60) con 100x130 m) dejaria
     sin suelo parte del recorte.
  4. El mundo se reescribe: fuera todos los <include>, dentro el modelo
     recortado + 4 paredes perimetrales en el borde (como el edificio de
     tugbot_warehouse: x=+-wx/2, y=+-wy/2, 0.5 m de grosor) + la camara cenital
     de tugbot (0,0,38, FOV 1.3). Fisica, plugins, luz y ground_plane se
     conservan tal cual.
  5. <actor>: si su trayectoria entra en la ventana (en frame mundo o relativa
     a su pose) se aborta, porque habria que trasladarla; si no, se quita.

Uso:
  /usr/bin/python3 scripts/crop_world.py rubber_factory --center -12.79 58.5 --size 30 50
Despues, borrar los modelos originales que ya no use ningun mundo.
"""
import argparse
import math
import os
import re
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_geometry as wg  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(REPO, "src", "swarm_worlds")
MODELS = os.path.join(PKG, "models")

WALL_T = 0.5     # grosor de pared (como el warehouse de tugbot)
WALL_H = 2.5     # alto: el LiDAR (z=0.557) la ve y no tapa la cenital
WALL_RGB = "0.62 0.62 0.6"


# ---- Corte de triangulos ---------------------------------------------------

def clip_tris_xy(tris, xmin, xmax, ymin, ymax):
    """Corta (N,3,3) contra el rectangulo XY. Devuelve (M,3,3)."""
    if len(tris) == 0:
        return tris
    lo, hi = tris[:, :, :2].min(1), tris[:, :, :2].max(1)
    out_ = (hi[:, 0] < xmin) | (lo[:, 0] > xmax) | (hi[:, 1] < ymin) | (lo[:, 1] > ymax)
    in_ = (lo[:, 0] >= xmin) & (hi[:, 0] <= xmax) & (lo[:, 1] >= ymin) & (hi[:, 1] <= ymax)
    keep = [tris[in_]]
    planes = ((0, 1, xmin), (0, -1, xmax), (1, 1, ymin), (1, -1, ymax))
    for tri in tris[~in_ & ~out_]:
        poly = list(tri)
        for ax, sign, lim in planes:
            res = []
            n = len(poly)
            for k in range(n):
                a, b = poly[k], poly[(k + 1) % n]
                da, db = sign * (a[ax] - lim), sign * (b[ax] - lim)
                if da >= 0:
                    res.append(a)
                if (da >= 0) != (db >= 0):
                    res.append(a + (da / (da - db)) * (b - a))
            poly = res
            if len(poly) < 3:
                break
        if len(poly) >= 3:
            p = np.array(poly)
            keep.append(np.stack([np.repeat(p[:1], len(p) - 2, 0), p[1:-1], p[2:]], 1))
    out = np.concatenate(keep) if keep else np.zeros((0, 3, 3))
    # Fuera triangulos degenerados (area ~0) que deja el corte en las aristas.
    area = np.linalg.norm(np.cross(out[:, 1] - out[:, 0], out[:, 2] - out[:, 0]), axis=1)
    return out[area > 1e-10]


def write_stl(path, tris):
    tris = tris.astype(np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"], rec["v"] = n, tris
    with open(path, "wb") as f:
        f.write(b"crop_world.py".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


# ---- Recorrido del mundo conservando el XML ---------------------------------

def _xml(el):
    import xml.etree.ElementTree as ET
    s = ET.tostring(el, encoding="unicode")
    return re.sub(r"\sxmlns(:\w+)?=\"[^\"]*\"", "", s).strip()


def collect(world_path):
    """[(model, link, kind, name, tris_mundo, xml_extra)] de todos los
    <include>. xml_extra = elementos a copiar literales (material, surface...)."""
    root = wg.read_xml(world_path)
    wel = wg._child(root, "world")
    items, actors = [], []
    for el in wel:
        if wg._tag(el) != "include":
            continue
        uri = wg._text(el, "uri")
        mdir = wg._resolve(uri, os.path.dirname(world_path), [MODELS])
        if mdir is None:
            sys.exit(f"no se encuentra {uri}")
        mroot = wg.read_xml(os.path.join(mdir, "model.sdf"))
        inner = wg._child(mroot, "model")
        name = wg._text(el, "name") or (inner.get("name") if inner is not None else uri)
        T = wg.pose_matrix(wg._text(el, "pose"))
        if inner is None:
            act = wg._child(mroot, "actor")
            if act is None:
                sys.exit(f"{uri}: ni <model> ni <actor>")
            actors.append(wg._parse_actor(act, T, name))
            continue
        for link in wg._children(inner, "link"):
            TL = T @ wg.pose_matrix(wg._text(link, "pose"))
            for kind in ("collision", "visual"):
                for c in wg._children(link, kind):
                    unres = []
                    shape, tris = wg._geometry(wg._child(c, "geometry"), mdir, [MODELS], unres)
                    if tris is None:
                        sys.exit(f"{name}/{c.get('name')}: geometria no cargada {unres}")
                    Tc = TL @ wg.pose_matrix(wg._text(c, "pose"))
                    keep = ("material", "transparency", "cast_shadows") if kind == "visual" \
                        else ("surface", "max_contacts", "laser_retro")
                    extra = "".join(_xml(x) for x in c if wg._tag(x) in keep)
                    items.append((name, link.get("name"), kind, c.get("name"),
                                  wg.transform(tris, Tc), extra))
    return items, actors


def walls_sdf(wx, wy):
    hx, hy = wx / 2.0, wy / 2.0
    segs = [("north", 0, hy, wx + WALL_T, WALL_T), ("south", 0, -hy, wx + WALL_T, WALL_T),
            ("east", hx, 0, WALL_T, wy + WALL_T), ("west", -hx, 0, WALL_T, wy + WALL_T)]
    body = "".join(f"""
        <collision name="wall_{n}_collision"><pose>{x} {y} {WALL_H / 2} 0 0 0</pose>
          <geometry><box><size>{sx} {sy} {WALL_H}</size></box></geometry></collision>
        <visual name="wall_{n}_visual"><pose>{x} {y} {WALL_H / 2} 0 0 0</pose>
          <geometry><box><size>{sx} {sy} {WALL_H}</size></box></geometry>
          <material><ambient>{WALL_RGB} 1</ambient><diffuse>{WALL_RGB} 1</diffuse></material></visual>"""
        for n, x, y, sx, sy in segs)
    return f"""
    <!-- Paredes perimetrales del recorte (crop_world.py): caras interiores en
         x=+-{hx - WALL_T / 2:g}, y=+-{hy - WALL_T / 2:g}. {WALL_H} m de alto: el LiDAR las
         ve y no tapan la cenital. -->
    <model name="perimeter_walls">
      <static>true</static>
      <link name="link">{body}
      </link>
    </model>"""


OVERHEAD = """
    <!-- Camara CENITAL, igual que en tugbot_warehouse: (0,0) a 38 m, FOV 1.3
         rad -> cubre ~58 m, el recorte de 30 x 50 m entero. -->
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
    </model>"""


def _shift_pose_text(text, cx, cy):
    v = [float(t) for t in (text or "").split()] + [0.0] * 6
    v = v[:6]
    v[0] -= cx
    v[1] -= cy
    return " ".join(f"{x:.6g}" for x in v)


def shift_inline(xml, cx, cy):
    """Traslada (-cx,-cy) cada <model> inline (salvo overhead_camera, que ya
    esta en el origen) y cada <light>: compone sobre su <pose> de primer nivel o
    la anade. Trabaja sobre el texto para conservar comentarios y formato."""
    def fix(m):
        block = m.group(0)
        if re.match(r"<model name=[\"']overhead_camera", block):
            return block
        open_tag = re.match(r"<(model|light)[^>]*>", block).group(0)
        body = block[len(open_tag):]
        # <pose> de PRIMER nivel: el que aparece antes de cualquier <link>.
        first_link = body.find("<link")
        pm = re.search(r"<pose[^>]*>([^<]*)</pose>", body)
        if pm and (first_link < 0 or pm.start() < first_link):
            new = f"<pose>{_shift_pose_text(pm.group(1), cx, cy)}</pose>"
            body = body[:pm.start()] + new + body[pm.end():]
        else:
            body = f"\n      <pose>{_shift_pose_text('', cx, cy)}</pose>" + body
        return open_tag + body

    return re.sub(r"<(model|light)\s[^>]*>.*?</\1>", fix, xml, flags=re.S)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world")
    ap.add_argument("--center", nargs=2, type=float, required=True)
    ap.add_argument("--size", nargs=2, type=float, default=(30.0, 50.0))
    args = ap.parse_args()
    cx, cy = args.center
    wx, wy = args.size
    world_path = os.path.join(PKG, "worlds", f"{args.world}.sdf")
    crop_name = f"{args.world}_crop"
    out_dir = os.path.join(MODELS, crop_name)
    os.makedirs(os.path.join(out_dir, "meshes"), exist_ok=True)

    items, actors = collect(world_path)
    box = (cx - wx / 2, cx + wx / 2, cy - wy / 2, cy + wy / 2)

    # Actores: abortar si pisan la ventana (en las dos interpretaciones).
    for a in actors:
        for _, Tw in a.waypoints:
            for p in (Tw[:2, 3], (a.pose @ Tw)[:2, 3]):
                if box[0] <= p[0] <= box[1] and box[2] <= p[1] <= box[3]:
                    sys.exit(f"el actor {a.name} entra en la ventana; no se recorta")
        print(f"actor {a.name}: fuera de la ventana -> se quita")

    shift = np.array([cx, cy, 0.0])
    cache, elems, stats = {}, [], []
    for model, link, kind, cname, tris, extra in items:
        key = tris.tobytes().__hash__()
        if key not in cache:
            cache[key] = clip_tris_xy(tris - shift, -wx / 2, wx / 2, -wy / 2, wy / 2)
        cut = cache[key]
        stats.append((model, kind, len(tris), len(cut)))
        if len(cut) == 0:
            continue
        fname = f"{model}__{link}__{cname}.stl".replace("/", "_")
        if kind == "visual":
            fname = fname.replace(".stl", "__vis.stl")
        write_stl(os.path.join(out_dir, "meshes", fname), cut)
        elems.append(f"""
      <{kind} name="{model}__{cname}">
        <geometry><mesh><uri>model://{crop_name}/meshes/{fname}</uri></mesh></geometry>
        {extra}
      </{kind}>""")

    with open(os.path.join(out_dir, "model.sdf"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<!-- GENERADO por scripts/crop_world.py a partir de worlds/{args.world}.sdf:
     ventana {wx:g} x {wy:g} m centrada en ({cx:g}, {cy:g}) del mundo original,
     trasladada al origen. No editar a mano. -->
<sdf version="1.8">
  <model name="{crop_name}">
    <static>true</static>
    <link name="link">{''.join(elems)}
    </link>
  </model>
</sdf>
""")
    with open(os.path.join(out_dir, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{crop_name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>Recorte {wx:g}x{wy:g} m de {args.world} (scripts/crop_world.py).</description>
</model>
""")

    # Mundo: fuera includes, cenital sustituida, dentro recorte + paredes.
    with open(world_path, encoding="utf-8") as f:
        xml = f.read()
    xml, n_inc = re.subn(r"\s*<include>.*?</include>", "", xml, flags=re.S)
    xml, n_cam = re.subn(r"\s*<!--[^>]*?CENITAL.*?-->\s*(?=<model name=[\"']overhead_camera)", "\n", xml, flags=re.S)
    xml, n_cam = re.subn(r"<model name=[\"']overhead_camera[\"']>.*?</model>", OVERHEAD.strip(), xml, flags=re.S)
    if n_cam != 1:
        sys.exit(f"se esperaba 1 overhead_camera, hay {n_cam}")
    inc = f"""
    <!-- Recorte {wx:g} x {wy:g} m del mundo original (scripts/crop_world.py) -->
    <include>
      <uri>model://{crop_name}</uri>
      <name>{crop_name}</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
{walls_sdf(wx, wy)}
"""
    xml = shift_inline(xml, cx, cy)
    head, sep, tail = xml.rpartition("</world>")
    xml = head.rstrip() + "\n" + inc + "\n  " + sep + tail
    with open(world_path, "w", encoding="utf-8") as f:
        f.write(xml)

    tot_in = sum(s[2] for s in stats)
    tot_out = sum(s[3] for s in stats)
    print(f"{n_inc} <include> sustituidos; triangulos {tot_in} -> {tot_out}")
    for m, k, a_, b_ in stats:
        if b_:
            print(f"  {m:40s} {k:9s} {a_:6d} -> {b_:6d}")
    print(f"descartados enteros: {sorted({m for m, k, a_, b_ in stats if b_ == 0} - {m for m, k, a_, b_ in stats if b_})}")


if __name__ == "__main__":
    main()
