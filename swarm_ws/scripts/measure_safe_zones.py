#!/usr/bin/env python3
"""Mide, SIN simular, la zona segura de grabacion de cada mundo.

Criterio de celda LIBRE (raster de RES m), todo desde la geometria real del SDF
(world_geometry.py, poses 6D completas):
  1. Sin COLISION en la banda z in [FLOOR_TOL, ROBOT_TOP]: nada contra lo que
     choque el robot, la pinza o la basura.
  2. Sin VISUAL en z in [FLOOR_TOL, ROBOT_TOP]: nada que el robot "atraviese"
     (un visual sin colision confunde al dataset: p.ej. las columnas
     'celling_collum' del warehouse son SOLO visuales). Y sin visual que tape
     la cenital en [ROBOT_TOP, CAM_Z], descartando como hace Ogre2 las caras
     que miran HACIA ABAJO (back-face culling: el techo 'roof' del warehouse
     tiene sus 8 triangulos con la normal hacia -z y desde arriba no se ve).
     Basta mirar en vertical sobre la zona: con el nadir de la camara dentro de
     la zona, la perspectiva proyecta lo de FUERA hacia fuera, nunca hacia
     dentro (r' = r*h/(h-z) > r).
  3. SUELO plano: hay soporte de colision con tope en [-FLOOR_TOL, FLOOR_TOL]
     (la basura se coloca a z=_TRASH_H/2 y el robot a z=0.15 suponiendo suelo 0).
  4. Lejos de la trayectoria de cualquier <actor> (ACTOR_CLEAR), probando la
     waypoint en frame mundo Y relativa a la pose del actor (conservador).

La zona es un CUADRADO alineado con los ejes: la imagen cenital es cuadrada, asi
que un cuadrado aprovecha todos sus pixeles y la escala m/px sale igual en todos
los mundos. El mayor cuadrado libre centrado en cada celda = distancia de
Chebyshev al obstaculo mas cercano (cv2 DIST_C, exacta en el raster).

La valla (de grosor FENCE_T) va en el borde de la zona y debe quedar a
FENCE_CLEAR de cualquier obstaculo: semilado libre exigido = half + FENCE_T +
FENCE_CLEAR.

Salidas:
  --report            tabla por mundo (siempre).
  --preview DIR       PNG por mundo: rojo=colision, naranja=solo visual,
                      gris=sin suelo, morado=actor, verde=zona, azul=spawn.
  --write             escribe en el paquete swarm_worlds:
                        config/safe_zones.yaml       (lo lee sim_summit.launch.py)
                        maps/<world>.pgm + .yaml     (occupancy, nav2 map_server)
                        maps/<world>_vector.yaml     (poligonos de obstaculos)
  --half H            semilado interior de la valla comun a todos los mundos
                      (por defecto: el mayor que cabe en TODOS, redondeado
                      hacia abajo a 0.25 m).

Uso:  /usr/bin/python3 scripts/measure_safe_zones.py --preview /tmp/zones [--write]
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
ACTOR_CLEAR = 1.0   # radio de exclusion alrededor de la trayectoria del actor
FENCE_T = 0.10      # grosor de la valla
FENCE_H = 0.30      # alto de la valla (por debajo del LiDAR, z=0.557)
FENCE_CLEAR = 0.20  # holgura entre la cara exterior de la valla y el mundo
SPAWN_MARGIN = 1.0  # de la cara interior de la valla al borde del spawn
CAM_Z = 12.0        # altura de la cenital sobre el suelo (igual en todos)
CAM_BORDER = 0.5    # franja visible mas alla de la valla en la cenital
MAP_MARGIN = 3.0    # el mapa guardado cubre la valla + este margen
PLANE_REACH = 12.0  # cuanto se mira mas alla de la geometria sobre el ground_plane
PREF_EXTRA = 0.30   # holgura extra preferida sobre FENCE_CLEAR al elegir centro

# Centro de REFERENCIA por mundo: el que se eligio y reviso a ojo con las
# camaras (CLAUDE.md, "Set de 5 mundos"). Se toma el centro factible MAS
# CERCANO a el, para conservar el decorado que se valido. Con el criterio de
# maxima holgura la zona se iba a explanadas vacias fuera del decorado.
ANCHORS = {
    "tugbot_warehouse": (0.0, 0.0),
    "colored_warehouse": (4.2, 4.2),
    "electrical_substation": (-36.0, 0.0),
    "rubber_factory": (-15.3, 85.5),
    "robotnik_lab": (3.7, -19.0),
    "empty_arena": (0.0, 0.0),
}


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
        rasterize_band(grid, vis[nz >= 0], ROBOT_TOP, CAM_Z, m_vis)
    top = floor_top(grid, col)
    m_nofloor = np.where(top >= -FLOOR_TOL, 0, 255).astype(np.uint8)
    m_act = actor_mask(grid, w)

    blocked = (m_col | m_vis | m_nofloor | m_act) > 0
    blocked[0, :] = blocked[-1, :] = blocked[:, 0] = blocked[:, -1] = True
    free = np.where(blocked, 0, 255).astype(np.uint8)
    # DIST_C: distancia de Chebyshev (en celdas) a la celda bloqueada mas cercana
    # = semilado del mayor cuadrado libre centrado en esa celda.
    cheb = cv2.distanceTransform(free, cv2.DIST_C, 3).astype(np.float32)
    return dict(name=name, world=w, grid=grid, m_col=m_col, m_vis=m_vis, m_vis_low=m_vis_low,
                m_nofloor=m_nofloor, m_act=m_act, cheb=cheb, top=top)


def free_half_max(a):
    """Semilado libre maximo (m) del mundo: (cheb - 0.5 celda) * RES."""
    return (float(a["cheb"].max()) - 0.5) * RES


def choose_center(a, half):
    """Centro para una valla de semilado interior half: la celda factible mas
    cercana al centro de referencia (ANCHORS). Factible = el cuadrado de la
    valla + FENCE_CLEAR esta libre; se prefieren las que ademas tienen
    PREF_EXTRA de holgura. Devuelve (x, y, semilado libre en esa celda)."""
    need = half + FENCE_T + FENCE_CLEAR
    cheb_m = (a["cheb"] - 0.5) * RES
    g = a["grid"]
    ax, ay = ANCHORS.get(a["name"], (0.0, 0.0))
    for req in (need + PREF_EXTRA, need):
        jj, ii = np.nonzero(cheb_m >= req)
        if len(ii):
            x, y = g.center(ii, jj)
            k = np.argmin((x - ax) ** 2 + (y - ay) ** 2)
            # Redondeo a 5 cm SIN salirse de la factibilidad: se vuelve a la
            # celda exacta si el redondeo la cambiara.
            return float(round(x[k], 2)), float(round(y[k], 2)), float(cheb_m[jj[k], ii[k]])
    return None


def verify_zone(a, cx, cy, half):
    """Re-comprueba la zona elegida directamente sobre las mascaras: devuelve
    cuantas celdas bloqueadas caen dentro del cuadrado de la valla + holgura
    (debe ser 0) y el rango de z del suelo en la zona."""
    g = a["grid"]
    need = half + FENCE_T + FENCE_CLEAR
    i0, j0 = g.ij(np.array(cx - need), np.array(cy - need))
    i1, j1 = g.ij(np.array(cx + need), np.array(cy + need))
    sl = (slice(int(j0), int(j1) + 1), slice(int(i0), int(i1) + 1))
    hits = {k: int((a[k][sl] > 0).sum()) for k in ("m_col", "m_vis", "m_nofloor", "m_act")}
    t = a["top"][sl]
    return hits, float(t.min()), float(t.max())


def camera_for(half):
    cover = 2.0 * (half + FENCE_T + CAM_BORDER)
    fov = 2.0 * math.atan(cover / 2.0 / CAM_Z)
    return CAM_Z, round(fov, 4), cover


# ---- Salidas ---------------------------------------------------------------

def preview(a, zone, path):
    g = a["grid"]
    img = np.full((g.h, g.w, 3), 255, np.uint8)
    img[a["m_nofloor"] > 0] = (170, 170, 170)
    img[a["m_vis"] > 0] = (0, 150, 255)
    img[a["m_col"] > 0] = (0, 0, 220)
    img[a["m_act"] > 0] = (200, 0, 160)
    if zone:
        cx, cy, half = zone
        for h, col, th in ((half + FENCE_T, (0, 160, 0), 3),
                           (half - SPAWN_MARGIN, (220, 120, 0), 2)):
            i0, j0 = g.ij(np.array(cx - h), np.array(cy - h))
            i1, j1 = g.ij(np.array(cx + h), np.array(cy + h))
            cv2.rectangle(img, (int(i0), int(j0)), (int(i1), int(j1)), col, th)
    img = np.flipud(img)  # fila 0 = y maxima (norte arriba)
    scale = max(1, int(1200 / max(g.w, g.h)))
    img = cv2.resize(img, (g.w * scale, g.h * scale), interpolation=cv2.INTER_NEAREST) \
        if scale > 1 else img
    if max(img.shape[:2]) > 2000:
        f = 2000 / max(img.shape[:2])
        img = cv2.resize(img, None, fx=f, fy=f, interpolation=cv2.INTER_AREA)
    cv2.imwrite(path, img)


def write_outputs(results, half):
    import yaml
    cfg_dir = os.path.join(PKG, "config")
    map_dir = os.path.join(PKG, "maps")
    os.makedirs(cfg_dir, exist_ok=True)
    os.makedirs(map_dir, exist_ok=True)
    cam_z, fov, cover = camera_for(half)
    zones = {}
    for a in results:
        cx, cy, clear = a["zone_center"]
        g = a["grid"]
        ext = half + FENCE_T + MAP_MARGIN
        x0, y0 = cx - ext, cy - ext
        i0, j0 = g.ij(np.array(x0), np.array(y0))
        n = int(round(2 * ext / RES))
        sl = (slice(int(j0), int(j0) + n), slice(int(i0), int(i0) + n))
        # Ocupado = colision O visual en la banda del robot (p.ej. las columnas
        # solo-visuales del warehouse) O sin suelo O trayectoria de actor.
        occ = ((a["m_col"][sl] > 0) | (a["m_vis_low"][sl] > 0)
               | (a["m_nofloor"][sl] > 0) | (a["m_act"][sl] > 0))
        ox, oy = g.center(int(i0), int(j0))
        ox, oy = float(ox - RES / 2), float(oy - RES / 2)
        # Occupancy (nav2): 0 = ocupado, 254 = libre. Fila 0 = y maxima.
        pgm = np.where(occ, 0, 254).astype(np.uint8)
        name = a["name"]
        cv2.imwrite(os.path.join(map_dir, f"{name}.pgm"), np.flipud(pgm))
        with open(os.path.join(map_dir, f"{name}.yaml"), "w") as f:
            f.write(f"image: {name}.pgm\nresolution: {RES}\n"
                    f"origin: [{ox:.3f}, {oy:.3f}, 0.0]\n"
                    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
                    "mode: trinary\n")
        # Mapa VECTORIAL: poligonos que ENVUELVEN los obstaculos. El contorno
        # de cv2 pasa por centros de celda (media celda DENTRO del obstaculo):
        # se dilata 1 celda antes (=> el contorno queda >= 0.5 celda fuera) y se
        # simplifica con error <= 0.5 celda => nunca corta el obstaculo y
        # sobra como mucho 1 celda (RES).
        # Se rellena PAD celdas alrededor para que lo que toca el borde del
        # recorte tambien pueda crecer hacia fuera.
        PAD = 2
        occ_p = cv2.copyMakeBorder(np.where(occ, 255, 0).astype(np.uint8),
                                   PAD, PAD, PAD, PAD, cv2.BORDER_REPLICATE)
        occ_d = cv2.dilate(occ_p, np.ones((3, 3), np.uint8))
        cnts, _ = cv2.findContours(occ_d, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        polys = []
        for c in cnts:
            c = cv2.approxPolyDP(c, 0.5, True).reshape(-1, 2)
            if len(c) < 3:
                x, y, bw, bh = cv2.boundingRect(c.reshape(-1, 1, 2))
                c = np.array([[x, y], [x + bw - 1, y], [x + bw - 1, y + bh - 1], [x, y + bh - 1]])
            pts_w = [[round(ox + (int(u) - PAD + 0.5) * RES, 3),
                      round(oy + (int(v) - PAD + 0.5) * RES, 3)]
                     for u, v in c]
            polys.append(pts_w)
        vec = {
            "world": name, "frame": "world", "units": "m",
            "note": "Generado por scripts/measure_safe_zones.py. Obstaculos = "
                    "colision o visual en z [%.2f, %.2f] m + sin suelo + actores, "
                    "recortado a la valla + %.1f m. Poligonos conservadores (envuelven "
                    "el obstaculo, exceso <= %.2f m). La valla NO va en 'obstacles'." % (
                        FLOOR_TOL, ROBOT_TOP, MAP_MARGIN, RES),
            "fence_inner": [round(cx - half, 3), round(cy - half, 3),
                            round(cx + half, 3), round(cy + half, 3)],
            "spawn_box": [round(cx - half + SPAWN_MARGIN, 3), round(cy - half + SPAWN_MARGIN, 3),
                          round(cx + half - SPAWN_MARGIN, 3), round(cy + half - SPAWN_MARGIN, 3)],
            "obstacles": polys,
        }
        with open(os.path.join(map_dir, f"{name}_vector.yaml"), "w") as f:
            yaml.safe_dump(vec, f, default_flow_style=None, sort_keys=False)
        zones[name] = {
            "world_name": a["world"].name,
            "center_x": cx, "center_y": cy,
            "fence_half": half,
            "spawn_half": round(half - SPAWN_MARGIN, 3),
            "max_free_half": round(a["max_half"], 2),
            "clearance": round(clear - (half + FENCE_T), 2),
        }
    doc = {
        "generated_by": "scripts/measure_safe_zones.py",
        "params": {"res": RES, "floor_tol": FLOOR_TOL, "robot_top": ROBOT_TOP,
                   "fence_t": FENCE_T, "fence_h": FENCE_H, "fence_clear": FENCE_CLEAR,
                   "spawn_margin": SPAWN_MARGIN, "actor_clear": ACTOR_CLEAR},
        "camera": {"z": cam_z, "fov": fov, "cover": round(cover, 3),
                   "width": 900, "height": 900},
        "worlds": zones,
    }
    with open(os.path.join(cfg_dir, "safe_zones.yaml"), "w") as f:
        f.write("# Zonas seguras de grabacion por mundo. NO editar a mano:\n"
                "#   /usr/bin/python3 scripts/measure_safe_zones.py --write\n")
        yaml.safe_dump(doc, f, sort_keys=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--worlds", nargs="*", default=WORLDS)
    ap.add_argument("--half", type=float, default=None)
    ap.add_argument("--preview", default=None)
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()

    results = []
    for name in args.worlds:
        a = analyze(name)
        a["max_half"] = free_half_max(a) - FENCE_T - FENCE_CLEAR
        results.append(a)
        unres = sorted(set(a["world"].unresolved))
        print(f"[{name}] raster {a['grid'].w}x{a['grid'].h} @ {RES} m; "
              f"semilado interior max de valla = {a['max_half']:.2f} m"
              + (f"; avisos: {unres}" if unres else ""))

    half = args.half
    if half is None:
        half = math.floor(min(a["max_half"] for a in results) / 0.25) * 0.25
    cam_z, fov, cover = camera_for(half)
    print(f"\nValla COMUN: semilado interior {half:.2f} m ({2*half:.2f} x {2*half:.2f} m), "
          f"spawn {2*(half-SPAWN_MARGIN):.2f} m. Cenital a {cam_z} m, FOV {fov} rad, "
          f"cubre {cover:.2f} m -> {900/cover:.0f} px/m (pieza 0.12 m = "
          f"{0.12*900/cover:.1f} px)\n")
    print(f"{'mundo':24s} {'centro':>18s} {'max':>6s} {'holgura':>8s}  verificacion")
    ok_all = True
    for a in results:
        z = choose_center(a, half)
        if z is None:
            print(f"{a['name']:24s} NO CABE (max {a['max_half']:.2f} m)")
            ok_all = False
            a["zone_center"] = None
            continue
        cx, cy, clear = z
        a["zone_center"] = z
        hits, zlo, zhi = verify_zone(a, cx, cy, half)
        ok = all(v == 0 for v in hits.values())
        ok_all &= ok
        print(f"{a['name']:24s} ({cx:7.2f},{cy:7.2f}) {a['max_half']:6.2f} "
              f"{clear - half - FENCE_T:7.2f}m  {'OK' if ok else 'FALLA'} {hits} "
              f"suelo z[{zlo:+.3f},{zhi:+.3f}]")
        if args.preview:
            os.makedirs(args.preview, exist_ok=True)
            preview(a, (cx, cy, half), os.path.join(args.preview, f"{a['name']}.png"))
    if args.write:
        if not ok_all:
            sys.exit("No se escribe nada: algun mundo no verifica.")
        write_outputs(results, half)
        print("\nEscrito config/safe_zones.yaml y maps/<mundo>{.pgm,.yaml,_vector.yaml}")


if __name__ == "__main__":
    main()
