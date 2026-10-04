#!/usr/bin/env python3
"""Mapas 2D de cada mundo (occupancy + vectorial) calculados SIN simular, desde
la geometria real del SDF (world_geometry.py: poses 6D, STL, COLLADA,
primitivas, actores; un <plane> de colision es infinito como en gz-physics).

Celda OCUPADA (raster de RES m):
  - colision en la banda z [FLOOR_TOL, ROBOT_TOP] (lo que choca con el robot,
    la pinza o la basura);
  - visual en esa misma banda (un visual SIN colision -p.ej. las columnas
    'celling_collum' de tugbot_warehouse- lo atraviesa el robot pero lo ve la
    camara: tampoco es sitio para grabar);
  - sin suelo de colision a |z| <= FLOOR_TOL;
  - trayectoria de un <actor> (+ACTOR_CLEAR), en frame mundo y relativa.

Salidas en src/swarm_worlds/maps/ (con --write):
  <mundo>.pgm + <mundo>.yaml   occupancy del mundo entero (nav2 map_server)
  <mundo>_vector.yaml          obstaculos como poligonos CON AGUJEROS ('outer'
                               + 'holes'; la zona libre son los agujeros), que
                               envuelven el obstaculo con exceso <= RES
Informe (siempre): hueco libre PARA EL ROBOT desde (0,0), que es el centro del
spawn por defecto del launch (area_half 8 necesita >= 8.45 m), y si algo tapa
la cenital encima del spawn (p.ej. las vigas a 10 m de tugbot_warehouse).

Tambien es la biblioteca de analisis que se uso para elegir las ventanas de
recorte de crop_world.py (analyze() + cheb).

Uso:  /usr/bin/python3 scripts/world_maps.py [--worlds ...] [--preview DIR] [--write]
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import world_geometry as wg  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(REPO, "src", "swarm_worlds")
MODELS = [os.path.join(PKG, "models")]

WORLDS = ["tugbot_warehouse", "colored_warehouse", "electrical_substation",
          "rubber_factory", "robotnik_lab", "empty_arena"]

RES = 0.05          # m/celda
FLOOR_TOL = 0.04    # |z| del suelo admitido; por encima = obstaculo
ROBOT_TOP = 2.0     # banda de colision (robot ~0.6 m + margen)
OVERHEAD_TOP = 40.0  # hasta donde se mira si algo tapa la cenital (esta a 38 m)
ACTOR_CLEAR = 1.0   # radio de exclusion alrededor de la trayectoria del actor
PLANE_REACH = 12.0  # cuanto se mira mas alla de la geometria sobre el ground_plane
MAP_MARGIN = 1.0    # el mapa guardado cubre la geometria (sin planos) + esto
SPAWN_NEED = 8.45   # area_half 8 + semiancho del robot


# ---- Rasterizacion ---------------------------------------------------------

class Grid:
    def __init__(self, x0, y0, x1, y1, res=RES):
        self.x0, self.y0, self.res = x0, y0, res
        self.w = int(math.ceil((x1 - x0) / res))
        self.h = int(math.ceil((y1 - y0) / res))

    def ij(self, x, y):
        return (np.floor((x - self.x0) / self.res).astype(np.int64),
                np.floor((y - self.y0) / self.res).astype(np.int64))

    def center(self, i, j):
        return self.x0 + (i + 0.5) * self.res, self.y0 + (j + 0.5) * self.res

    def zeros(self, dtype=np.uint8, fill=0):
        return np.full((self.h, self.w), fill, dtype=dtype)


def _clip_slab(tri, z0, z1):
    """Sutherland-Hodgman del triangulo (3,3) contra z0 <= z <= z1."""
    poly = list(tri)
    for sign, lim in ((1, z0), (-1, z1)):
        out = []
        n = len(poly)
        for k in range(n):
            a, b = poly[k], poly[(k + 1) % n]
            da, db = sign * (a[2] - lim), sign * (b[2] - lim)
            if da >= 0:
                out.append(a)
            if (da >= 0) != (db >= 0):
                t = da / (da - db)
                out.append(a + t * (b - a))
        poly = out
        if not poly:
            return None
    return np.array(poly)


def rasterize_band(grid, tris, z0, z1, mask=None):
    """Marca (255) toda celda que toque la parte de algun triangulo dentro de
    la banda z0..z1. Conservador: los triangulos menores que una celda marcan
    su bbox entera; los grandes se recortan a la banda y se rellenan + borde."""
    if mask is None:
        mask = grid.zeros()
    if len(tris) == 0:
        return mask
    zmin, zmax = tris[:, :, 2].min(1), tris[:, :, 2].max(1)
    tris = tris[(zmax >= z0) & (zmin <= z1)]
    if len(tris) == 0:
        return mask
    xy = tris[:, :, :2]
    ext = (xy.max(1) - xy.min(1)).max(1)
    small = ext < grid.res
    s = xy[small]
    if len(s):
        lo, hi = s.min(1), s.max(1)
        for cx, cy in ((lo[:, 0], lo[:, 1]), (lo[:, 0], hi[:, 1]),
                       (hi[:, 0], lo[:, 1]), (hi[:, 0], hi[:, 1])):
            i, j = grid.ij(cx, cy)
            ok = (i >= 0) & (i < grid.w) & (j >= 0) & (j < grid.h)
            mask[j[ok], i[ok]] = 255
    SH = 4
    for tri in tris[~small]:
        poly = _clip_slab(tri, z0, z1)
        if poly is None or len(poly) < 3:
            continue
        uv = ((poly[:, :2] - [grid.x0, grid.y0]) / grid.res - 0.5) * (1 << SH)
        pts = np.round(uv).astype(np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(mask, [pts], 255, lineType=cv2.LINE_8, shift=SH)
        cv2.polylines(mask, [pts], True, 255, 1, lineType=cv2.LINE_8, shift=SH)
    return mask


def floor_top(grid, tris):
    """Tope del suelo por celda: max z (<= FLOOR_TOL) de la colision, muestreado
    en el centro de cada celda por interpolacion baricentrica. -inf = sin suelo."""
    top = grid.zeros(np.float32, -np.inf)
    zmin = tris[:, :, 2].min(1)
    tris = tris[zmin <= FLOOR_TOL]
    for tri in tris:
        a, b, c = tri
        lo, hi = tri[:, :2].min(0), tri[:, :2].max(0)
        i0, j0 = grid.ij(lo[0], lo[1])
        i1, j1 = grid.ij(hi[0], hi[1])
        i0, j0 = max(int(i0), 0), max(int(j0), 0)
        i1, j1 = min(int(i1), grid.w - 1), min(int(j1), grid.h - 1)
        if i1 < i0 or j1 < j0:
            continue
        ii, jj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1))
        px, py = grid.center(ii, jj)
        v0, v1 = b[:2] - a[:2], c[:2] - a[:2]
        den = v0[0] * v1[1] - v1[0] * v0[1]
        if abs(den) < 1e-12:   # triangulo vertical: no es suelo
            continue
        dx, dy = px - a[0], py - a[1]
        l1 = (dx * v1[1] - v1[0] * dy) / den
        l2 = (v0[0] * dy - dx * v0[1]) / den
        inside = (l1 >= -1e-9) & (l2 >= -1e-9) & (l1 + l2 <= 1 + 1e-9)
        z = a[2] + l1 * (b[2] - a[2]) + l2 * (c[2] - a[2])
        z = np.where(inside & (z <= FLOOR_TOL), z, -np.inf).astype(np.float32)
        sub = top[j0:j1 + 1, i0:i1 + 1]
        np.maximum(sub, z, out=sub)
    return top


def actor_mask(grid, world):
    m = grid.zeros()
    r = int(math.ceil(ACTOR_CLEAR / grid.res))
    for a in world.actors:
        for frame in ("mundo", "relativa"):
            pts = []
            for _, T in a.waypoints:
                p = (a.pose @ T)[:2, 3] if frame == "relativa" else T[:2, 3]
                i, j = grid.ij(np.array(p[0]), np.array(p[1]))
                pts.append([int(i), int(j)])
            if pts:
                cv2.polylines(m, [np.array(pts, np.int32).reshape(-1, 1, 2)],
                              False, 255, 2 * r + 1)
    return m


# ---- Analisis por mundo ----------------------------------------------------

def analyze(name):
    w = wg.load_world(os.path.join(PKG, "worlds", f"{name}.sdf"), MODELS)
    # Extension del raster: la geometria (no planos) + 2 m, ampliada con los
    # planos de suelo pero sin pasar de PLANE_REACH alrededor de la geometria
    # (un ground_plane de 100x100 m no debe disparar el raster).
    pts = np.concatenate([g.tris for g in w.geoms if g.shape != "plane"]).reshape(-1, 3)
    lo, hi = pts[:, :2].min(0) - 2.0, pts[:, :2].max(0) + 2.0
    planes = [g.tris for g in w.geoms if g.shape == "plane"]
    if planes:
        pp = np.concatenate(planes).reshape(-1, 3)[:, :2]
        lo = np.maximum(np.minimum(lo, pp.min(0)), lo - PLANE_REACH)
        hi = np.minimum(np.maximum(hi, pp.max(0)), hi + PLANE_REACH)
    grid = Grid(lo[0], lo[1], hi[0], hi[1])

    col = np.concatenate([g.tris for g in w.geoms if g.kind == "collision"])
    vis = [g.tris for g in w.geoms if g.kind == "visual"]
    vis = np.concatenate(vis) if vis else np.zeros((0, 3, 3))

    m_col = rasterize_band(grid, col, FLOOR_TOL, ROBOT_TOP)
    m_vis = rasterize_band(grid, vis, FLOOR_TOL, ROBOT_TOP)
    m_vis_low = m_vis.copy()   # visual en la banda del robot (para el mapa)
    if len(vis):
        nz = np.cross(vis[:, 1] - vis[:, 0], vis[:, 2] - vis[:, 0])[:, 2]
        rasterize_band(grid, vis[nz >= 0], ROBOT_TOP, OVERHEAD_TOP, m_vis)
    top = floor_top(grid, col)
    m_nofloor = np.where(top >= -FLOOR_TOL, 0, 255).astype(np.uint8)
    m_act = actor_mask(grid, w)

    blocked = (m_col | m_vis | m_nofloor | m_act) > 0
    blocked[0, :] = blocked[-1, :] = blocked[:, 0] = blocked[:, -1] = True
    free = np.where(blocked, 0, 255).astype(np.uint8)
    # DIST_C: distancia de Chebyshev (en celdas) a la celda bloqueada mas cercana
    # = semilado del mayor cuadrado libre centrado en esa celda.
    cheb = cv2.distanceTransform(free, cv2.DIST_C, 3).astype(np.float32)
    # Lo mismo SOLO para el robot (sin lo que tapa la cenital desde arriba).
    rb = (m_col | m_vis_low | m_nofloor | m_act) > 0
    rb[0, :] = rb[-1, :] = rb[:, 0] = rb[:, -1] = True
    cheb_robot = cv2.distanceTransform(np.where(rb, 0, 255).astype(np.uint8),
                                       cv2.DIST_C, 3).astype(np.float32)
    return dict(cheb_robot=cheb_robot, name=name, world=w, grid=grid, m_col=m_col, m_vis=m_vis, m_vis_low=m_vis_low,
                m_nofloor=m_nofloor, m_act=m_act, cheb=cheb, top=top)


def free_at(a, x, y, key="cheb_robot"):
    """Semilado (m) del mayor cuadrado libre centrado en (x, y)."""
    i, j = a["grid"].ij(np.array(x), np.array(y))
    return (float(a[key][int(j), int(i)]) - 0.5) * RES


def preview(a, path):
    """PNG: rojo=colision, naranja=solo visual, gris=sin suelo, morado=actor."""
    g = a["grid"]
    img = np.full((g.h, g.w, 3), 255, np.uint8)
    img[a["m_nofloor"] > 0] = (170, 170, 170)
    img[a["m_vis"] > 0] = (0, 150, 255)
    img[a["m_col"] > 0] = (0, 0, 220)
    img[a["m_act"] > 0] = (200, 0, 160)
    img = np.flipud(img)  # fila 0 = y maxima (norte arriba)
    f = 1400 / max(img.shape[:2])
    cv2.imwrite(path, cv2.resize(img, None, fx=f, fy=f, interpolation=cv2.INTER_AREA))


def write_maps(a):
    import yaml
    g, w = a["grid"], a["world"]
    pts = np.concatenate([x.tris for x in w.geoms if x.shape != "plane"]).reshape(-1, 3)
    lo, hi = pts[:, :2].min(0) - MAP_MARGIN, pts[:, :2].max(0) + MAP_MARGIN
    i0, j0 = (int(v) for v in g.ij(np.array(lo[0]), np.array(lo[1])))
    i1, j1 = (int(v) for v in g.ij(np.array(hi[0]), np.array(hi[1])))
    i0, j0 = max(i0, 0), max(j0, 0)
    i1, j1 = min(i1, g.w - 1), min(j1, g.h - 1)
    sl = (slice(j0, j1 + 1), slice(i0, i1 + 1))
    occ = ((a["m_col"][sl] > 0) | (a["m_vis_low"][sl] > 0)
           | (a["m_nofloor"][sl] > 0) | (a["m_act"][sl] > 0))
    ox, oy = g.center(i0, j0)
    ox, oy = float(ox - RES / 2), float(oy - RES / 2)
    name = a["name"]
    out = os.path.join(PKG, "maps")
    os.makedirs(out, exist_ok=True)
    # Occupancy (nav2): 0 = ocupado, 254 = libre. Fila 0 = y maxima.
    cv2.imwrite(os.path.join(out, f"{name}.pgm"), np.flipud(np.where(occ, 0, 254).astype(np.uint8)))
    with open(os.path.join(out, f"{name}.yaml"), "w") as f:
        f.write(f"image: {name}.pgm\nresolution: {RES}\n"
                f"origin: [{ox:.3f}, {oy:.3f}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n")
    # Vectorial: el contorno de cv2 pasa por centros de celda (media celda
    # DENTRO del obstaculo). Se dilata 1 celda (=> >= 0.5 celda fuera) y se
    # simplifica con error <= 0.5 celda: nunca corta el obstaculo y sobra como
    # mucho RES. PAD celdas de relleno para que lo que toca el borde crezca.
    PAD = 2
    occ_p = cv2.copyMakeBorder(np.where(occ, 255, 0).astype(np.uint8),
                               PAD, PAD, PAD, PAD, cv2.BORDER_REPLICATE)
    occ_d = cv2.dilate(occ_p, np.ones((3, 3), np.uint8))
    # RETR_CCOMP: dos niveles -> contorno exterior de cada region ocupada y
    # sus agujeros (zona libre rodeada de obstaculo, p.ej. el interior de un
    # edificio cerrado por paredes).
    cnts, hier = cv2.findContours(occ_d, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    def to_world(c):
        c = cv2.approxPolyDP(c, 0.5, True).reshape(-1, 2)
        if len(c) < 3:
            x, y, bw, bh = cv2.boundingRect(c.reshape(-1, 1, 2))
            c = np.array([[x, y], [x + bw - 1, y], [x + bw - 1, y + bh - 1], [x, y + bh - 1]])
        return [[round(ox + (int(u) - PAD + 0.5) * RES, 3),
                 round(oy + (int(v) - PAD + 0.5) * RES, 3)] for u, v in c]

    polys = []
    if hier is not None:
        for k, h in enumerate(hier[0]):
            if h[3] != -1:
                continue                      # es un agujero: va con su padre
            holes, ch = [], h[2]
            while ch != -1:
                holes.append(to_world(cnts[ch]))
                ch = hier[0][ch][0]
            polys.append({"outer": to_world(cnts[k]), "holes": holes})
    vec = {
        "world": name, "frame": "world", "units": "m",
        "note": ("Generado por scripts/world_maps.py. Obstaculos = colision o visual en "
                 f"z [{FLOOR_TOL}, {ROBOT_TOP}] m + sin suelo + actores. Poligonos "
                 f"conservadores (envuelven el obstaculo, exceso <= {RES} m)."),
        "extent": [round(ox, 3), round(oy, 3),
                   round(ox + (i1 - i0 + 1) * RES, 3), round(oy + (j1 - j0 + 1) * RES, 3)],
        "obstacles": polys,
    }
    with open(os.path.join(out, f"{name}_vector.yaml"), "w") as f:
        yaml.safe_dump(vec, f, default_flow_style=None, sort_keys=False)
    return occ.shape, len(polys)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--worlds", nargs="*", default=WORLDS)
    ap.add_argument("--preview", default=None)
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()
    print(f"{'mundo':24s} {'robot libre desde (0,0)':>32s}  {'cenital tapada sobre spawn':>27s}  mapa")
    for name in args.worlds:
        a = analyze(name)
        f0 = free_at(a, 0.0, 0.0)
        g = a["grid"]
        i0, j0 = (int(v) for v in g.ij(np.array(-SPAWN_NEED), np.array(-SPAWN_NEED)))
        i1, j1 = (int(v) for v in g.ij(np.array(SPAWN_NEED), np.array(SPAWN_NEED)))
        over = int(((a["m_vis"] > 0) & ~(a["m_vis_low"] > 0))[j0:j1 + 1, i0:i1 + 1].sum())
        msg = ""
        if args.write:
            shape, npoly = write_maps(a)
            msg = f"{shape[1]}x{shape[0]} celdas, {npoly} poligonos"
        if args.preview:
            os.makedirs(args.preview, exist_ok=True)
            preview(a, os.path.join(args.preview, f"{name}.png"))
        ok = "OK spawn por defecto" if f0 >= SPAWN_NEED else "pasar center/area_half"
        print(f"{name:24s} {f0:7.2f} m {ok:>23s}  {over * RES * RES:21.1f} m2  {msg}")
        unres = sorted(set(a["world"].unresolved))
        if unres:
            print(f"    avisos: {unres}")


if __name__ == "__main__":
    main()
