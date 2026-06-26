#!/usr/bin/env python3
"""Genera un occupancy grid 2D estatico del mundo tugbot_warehouse SIN simular.

Lee los <include> del mundo (nombre, uri, pose), localiza cada modelo (warehouse
vendorizado en el repo + modelos MovAi en el cache de Fuel: ~/.gz/fuel en
Harmonic, ~/.ignition/fuel en Fortress),
compone la pose mundo -> link -> colision de cada <collision>, y rasteriza su
huella (box / cylinder / mesh-AABB) en un PGM, filtrando por la franja de altura
del LiDAR (Z_BAND) para excluir el suelo y los obstaculos bajos.

Salida: swarm_ws/src/swarm_worlds/maps/warehouse.pgm + warehouse.yaml
(formato nav2 map_server).

Uso:  python3 scripts/make_warehouse_map.py
"""
import math
import os
import struct
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(REPO, "src/swarm_worlds/worlds/tugbot_warehouse.sdf")
MODELS_DIR = os.path.join(REPO, "src/swarm_worlds/models")
def _find_fuel():
    """Cache de Fuel de los modelos MovAi. Harmonic usa ~/.gz/fuel y el server
    fuel.gazebosim.org; Fortress usaba ~/.ignition/fuel y fuel.ignitionrobotics.org.
    Devuelve el primer .../movai/models que exista."""
    candidates = [
        "~/.gz/fuel/fuel.gazebosim.org/movai/models",
        "~/.ignition/fuel/fuel.gazebosim.org/movai/models",
        "~/.gz/fuel/fuel.ignitionrobotics.org/movai/models",
        "~/.ignition/fuel/fuel.ignitionrobotics.org/movai/models",
    ]
    for c in candidates:
        p = os.path.expanduser(c)
        if os.path.isdir(p):
            return p
    # Si nada existe aún (1ª ejecución sin cache), devuelve la ruta Harmonic.
    return os.path.expanduser(candidates[0])


FUEL = _find_fuel()
OUT_DIR = os.path.join(REPO, "src/swarm_worlds/maps")

# Plano del mundo: limites y resolucion del grid.
X_MIN, X_MAX = -16.0, 16.0
Y_MIN, Y_MAX = -26.0, 26.0
RES = 0.05                       # m/celda
# Franja de altura (mundo) que "ve" el LiDAR 2D: excluye suelo (~0.01 m) y
# pallets/rieles bajos, incluye estanterias, paredes, cajas altas y carros.
Z_BAND = (0.20, 0.70)

# Fuel URI (cola) -> ruta del model.sdf cacheado (version mas alta disponible).
FUEL_MAP = {
    "shelf": "shelf",
    "shelf_big": "shelf_big",
    "cart_model_2": "cart_model_2",
    "pallet_box_mobile": "pallet_box_mobile",
    "Tugbot-charging-station": "tugbot-charging-station",
}

_S = lambda t: t.split("}")[-1]   # quita el namespace XML


def _pose(text):
    """'x y z r p yaw' -> (x, y, z, yaw). Ignora roll/pitch (0 en este mundo)."""
    if not text:
        return (0.0, 0.0, 0.0, 0.0)
    v = [float(x) for x in text.split()]
    v += [0.0] * (6 - len(v))
    return (v[0], v[1], v[2], v[5])


def _compose(parent, child):
    """Compone dos poses (x,y,z,yaw): child expresada en el frame de parent."""
    px, py, pz, pyaw = parent
    cx, cy, cz, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return (px + c * cx - s * cy, py + s * cx + c * cy, pz + cz, pyaw + cyaw)


def _fuel_sdf(tail):
    base = os.path.join(FUEL, FUEL_MAP[tail])
    versions = sorted((d for d in os.listdir(base) if d.isdigit()), key=int)
    return os.path.join(base, versions[-1], "model.sdf")


def _stl_aabb(path):
    """AABB XY y Z de un STL binario. Devuelve (sx, sy, zc, zh) o None."""
    with open(path, "rb") as f:
        f.read(80)
        (n,) = struct.unpack("<I", f.read(4))
        xs, ys, zs = [], [], []
        for _ in range(n):
            f.read(12)  # normal
            for _ in range(3):
                x, y, z = struct.unpack("<3f", f.read(12))
                xs.append(x); ys.append(y); zs.append(z)
            f.read(2)
    if not xs:
        return None
    return (max(xs) - min(xs), max(ys) - min(ys),
            (max(zs) + min(zs)) / 2.0, (max(zs) - min(zs)) / 2.0)


def _z_hits(zc, zh):
    """¿El intervalo [zc-zh, zc+zh] corta la franja del LiDAR?"""
    return (zc + zh) >= Z_BAND[0] and (zc - zh) <= Z_BAND[1]


def collect_collisions():
    """Devuelve lista de huellas a pintar: ('box',cx,cy,sx,sy,yaw) o
    ('cyl',cx,cy,r)."""
    with open(WORLD, "r", encoding="utf-8", errors="replace") as f:
        tree = ET.fromstring(f.read())
    foots = []
    for inc in tree.iter():
        if _S(inc.tag) != "include":
            continue
        uri = (inc.findtext("{*}uri") or "").strip()
        wpose = _pose(inc.findtext("{*}pose"))
        tail = uri.rstrip("/").split("/")[-1]
        if uri.startswith("model://warehouse"):
            sdf = os.path.join(MODELS_DIR, "warehouse", "model.sdf")
        elif tail in FUEL_MAP:
            sdf = _fuel_sdf(tail)
        else:
            print(f"  ?? include sin mapear: {uri}")
            continue
        model_dir = os.path.dirname(sdf)
        foots += _model_footprints(sdf, model_dir, wpose)
    return foots


def _model_footprints(sdf, model_dir, wpose):
    root = ET.parse(sdf).getroot()
    out = []
    for link in root.iter():
        if _S(link.tag) != "link":
            continue
        lpose = _pose(link.findtext("{*}pose"))
        for col in link.iter():
            if _S(col.tag) != "collision":
                continue
            cpose = _pose(col.findtext("{*}pose"))
            center = _compose(_compose(wpose, lpose), cpose)
            cx, cy, cz, yaw = center
            geom = col.find(".//{*}geometry")
            if geom is None:
                continue
            box = geom.find("{*}box/{*}size")
            cyl = geom.find("{*}cylinder")
            mesh = geom.find("{*}mesh")
            if box is not None:
                sx, sy, sz = [float(v) for v in box.text.split()]
                if _z_hits(cz, sz / 2.0):
                    out.append(("box", cx, cy, sx, sy, yaw))
            elif cyl is not None:
                r = float(cyl.findtext("{*}radius"))
                lz = float(cyl.findtext("{*}length"))
                if _z_hits(cz, lz / 2.0):
                    out.append(("cyl", cx, cy, r))
            elif mesh is not None:
                uri = (mesh.findtext("{*}uri") or "").strip()
                mp = os.path.join(model_dir, uri.replace("model://", "").split("/", 1)[-1]) \
                    if uri.startswith("model://") else os.path.join(model_dir, uri)
                if os.path.isfile(mp) and mp.lower().endswith(".stl"):
                    aabb = _stl_aabb(mp)
                    if aabb and _z_hits(cz + aabb[2], aabb[3]):
                        out.append(("box", cx, cy, aabb[0], aabb[1], yaw))
    return out


def rasterize(foots):
    W = int(round((X_MAX - X_MIN) / RES))
    H = int(round((Y_MAX - Y_MIN) / RES))
    grid = bytearray([254]) * (W * H)  # 254 = libre

    def stamp(ix, iy):
        if 0 <= ix < W and 0 <= iy < H:
            grid[iy * W + ix] = 0  # 0 = ocupado

    for f in foots:
        if f[0] == "box":
            _, cx, cy, sx, sy, yaw = f
            rad = math.hypot(sx, sy) / 2.0
            c, s = math.cos(yaw), math.sin(yaw)
            ix0 = int((cx - rad - X_MIN) / RES); ix1 = int((cx + rad - X_MIN) / RES)
            iy0 = int((cy - rad - Y_MIN) / RES); iy1 = int((cy + rad - Y_MIN) / RES)
            for iy in range(iy0, iy1 + 1):
                wy = Y_MIN + (iy + 0.5) * RES
                for ix in range(ix0, ix1 + 1):
                    wx = X_MIN + (ix + 0.5) * RES
                    dx, dy = wx - cx, wy - cy
                    lx = c * dx + s * dy        # a frame de la caja (-yaw)
                    ly = -s * dx + c * dy
                    if abs(lx) <= sx / 2.0 and abs(ly) <= sy / 2.0:
                        stamp(ix, iy)
        else:  # cyl
            _, cx, cy, r = f
            ix0 = int((cx - r - X_MIN) / RES); ix1 = int((cx + r - X_MIN) / RES)
            iy0 = int((cy - r - Y_MIN) / RES); iy1 = int((cy + r - Y_MIN) / RES)
            for iy in range(iy0, iy1 + 1):
                wy = Y_MIN + (iy + 0.5) * RES
                for ix in range(ix0, ix1 + 1):
                    wx = X_MIN + (ix + 0.5) * RES
                    if math.hypot(wx - cx, wy - cy) <= r:
                        stamp(ix, iy)
    return grid, W, H


def write_pgm(grid, W, H, path):
    # PGM: fila 0 = arriba = y maximo, asi que se escribe de Y_MAX a Y_MIN.
    with open(path, "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode())
        for iy in range(H - 1, -1, -1):
            f.write(bytes(grid[iy * W:(iy + 1) * W]))


def write_yaml(path, pgm_name):
    with open(path, "w") as f:
        f.write(
            f"image: {pgm_name}\n"
            f"resolution: {RES}\n"
            f"origin: [{X_MIN}, {Y_MIN}, 0.0]\n"
            f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
        )


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    foots = collect_collisions()
    print(f"huellas a rasterizar: {len(foots)} "
          f"(box={sum(1 for f in foots if f[0]=='box')}, "
          f"cyl={sum(1 for f in foots if f[0]=='cyl')})")
    grid, W, H = rasterize(foots)
    occ = sum(1 for b in grid if b == 0)
    pgm = os.path.join(OUT_DIR, "warehouse.pgm")
    write_pgm(grid, W, H, pgm)
    write_yaml(os.path.join(OUT_DIR, "warehouse.yaml"), "warehouse.pgm")
    print(f"grid {W}x{H} @ {RES} m, ocupadas={occ} ({100*occ/(W*H):.1f}%)")
    print(f"escrito: {pgm}")


if __name__ == "__main__":
    main()
