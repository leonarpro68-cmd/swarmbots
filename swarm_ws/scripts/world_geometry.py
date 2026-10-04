"""Geometria de un mundo SDF en frame MUNDO, sin simular.

Carga TODAS las <collision> y <visual> de un mundo (modelos inline e
<include> de model://) como triangulos en frame mundo, componiendo poses 6D
completas (roll/pitch incluidos: varios STL de Robotnik vienen con roll=1.57).

Soporta: box, cylinder, sphere, plane (finito, con su <size>) y mesh STL
(binario/ASCII) y COLLADA .dae (nodos <matrix>/<translate>/<rotate>/<scale>,
<triangles>/<polylist>, <unit>; Z_UP o Y_UP). Los <actor> se devuelven aparte
con su trayectoria (son obstaculos que se MUEVEN).

Lo usa scripts/measure_safe_zones.py. Necesita numpy (usar /usr/bin/python3
si el python3 por defecto es un conda sin numpy).
"""
import math
import os
import re
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np


def read_xml(path):
    """Parsea un SDF/DAE tolerando la declaracion 'encoding=ASCII' falsa
    (tugbot_warehouse.sdf la declara y contiene UTF-8)."""
    with open(path, "rb") as f:
        txt = f.read().decode("utf-8", errors="replace")
    txt = re.sub(r"<\?xml[^>]*\?>", "", txt, count=1)
    return ET.fromstring(txt)


def _tag(el):
    return el.tag.split("}")[-1]


def _child(el, name):
    for c in el:
        if _tag(c) == name:
            return c
    return None


def _children(el, name):
    return [c for c in el if _tag(c) == name]


def _text(el, name, default=None):
    c = _child(el, name)
    return c.text.strip() if c is not None and c.text else default


# ---- Poses ---------------------------------------------------------------

def rpy_matrix(r, p, y):
    """R = Rz(y) Ry(p) Rx(r), convencion SDF."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def pose_matrix(text):
    """'x y z r p y' -> 4x4. Vacio/None -> identidad."""
    T = np.eye(4)
    if not text:
        return T
    v = [float(t) for t in text.split()]
    v += [0.0] * (6 - len(v))
    T[:3, :3] = rpy_matrix(v[3], v[4], v[5])
    T[:3, 3] = v[:3]
    return T


def transform(tris, T):
    """(N,3,3) -> (N,3,3) aplicando la 4x4 T."""
    if len(tris) == 0:
        return tris
    return tris @ T[:3, :3].T + T[:3, 3]


# ---- Primitivas ------------------------------------------------------------

def box_tris(sx, sy, sz):
    h = np.array([sx, sy, sz]) / 2.0
    v = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * h
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
             (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return v[np.array(faces)]


def cylinder_tris(r, length, n=32):
    """Prisma de n lados CIRCUNSCRITO al cilindro (conservador: nunca mas
    pequeno que el cilindro real)."""
    rr = r / math.cos(math.pi / n)
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.stack([rr * np.cos(a), rr * np.sin(a)], axis=1)
    h = length / 2.0
    tris = []
    for i in range(n):
        j = (i + 1) % n
        p0, p1 = ring[i], ring[j]
        b0, b1 = [*p0, -h], [*p1, -h]
        t0, t1 = [*p0, h], [*p1, h]
        tris += [[b0, b1, t1], [b0, t1, t0],
                 [[0, 0, -h], b1, b0], [[0, 0, h], t0, t1]]
    return np.array(tris, dtype=float)


def sphere_tris(r, n=16):
    """Esfera lat-long; se escala para quedar circunscrita (conservador)."""
    rr = r / math.cos(math.pi / n) ** 2
    th = np.linspace(0, math.pi, n + 1)
    ph = np.linspace(0, 2 * math.pi, 2 * n + 1)
    P = np.array([[[rr * math.sin(t) * math.cos(f), rr * math.sin(t) * math.sin(f),
                    rr * math.cos(t)] for f in ph] for t in th])
    tris = []
    for i in range(n):
        for j in range(2 * n):
            a, b, c, d = P[i, j], P[i, j + 1], P[i + 1, j + 1], P[i + 1, j]
            tris += [[a, b, c], [a, c, d]]
    return np.array(tris)


def plane_tris(normal, size):
    """Plano FINITO de lado size centrado en el origen, orientado a normal."""
    n = np.array(normal, dtype=float)
    n /= np.linalg.norm(n)
    sx, sy = size
    v = np.array([[-sx, -sy, 0], [sx, -sy, 0], [sx, sy, 0], [-sx, sy, 0]]) / 2.0
    z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(z, n)
    s, c = np.linalg.norm(axis), float(np.dot(z, n))
    if s > 1e-9:
        k = axis / s
        K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        R = np.eye(3) + s * K + (1 - c) * K @ K
        v = v @ R.T
    return v[np.array([(0, 1, 2), (0, 2, 3)])]


# ---- Mallas ----------------------------------------------------------------

_MESH_CACHE = {}


def load_stl(path):
    with open(path, "rb") as f:
        data = f.read()
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + 50 * n == len(data):
            rec = np.frombuffer(data, dtype=np.dtype([
                ("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                count=n, offset=84)
            return rec["v"].astype(float)
    # ASCII
    vs = [list(map(float, m.groups())) for m in re.finditer(
        rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)]
    return np.array(vs, dtype=float).reshape(-1, 3, 3)


def _dae_node_matrix(node):
    M = np.eye(4)
    for t in node:
        k = _tag(t)
        vals = [float(x) for x in (t.text or "").split()] if k in (
            "matrix", "translate", "rotate", "scale") else None
        if k == "matrix":
            M = M @ np.array(vals).reshape(4, 4)
        elif k == "translate":
            T = np.eye(4)
            T[:3, 3] = vals
            M = M @ T
        elif k == "rotate":
            ax = np.array(vals[:3])
            ax /= np.linalg.norm(ax)
            a = math.radians(vals[3])
            K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
            R = np.eye(4)
            R[:3, :3] = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
            M = M @ R
        elif k == "scale":
            M = M @ np.diag([*vals, 1.0])
    return M


def load_dae(path, submesh=None):
    """Triangulos de un COLLADA. submesh: solo el nodo/geometria con ese
    nombre (como <submesh><name> de SDF)."""
    root = read_xml(path)
    asset = _child(root, "asset")
    unit, up = 1.0, "Z_UP"
    if asset is not None:
        u = _child(asset, "unit")
        if u is not None and u.get("meter"):
            unit = float(u.get("meter"))
        up = _text(asset, "up_axis", "Z_UP")

    geoms = {}
    for lib in root.iter():
        if _tag(lib) != "library_geometries":
            continue
        for g in _children(lib, "geometry"):
            mesh = _child(g, "mesh")
            if mesh is None:
                continue
            sources = {}
            for s in _children(mesh, "source"):
                fa = _child(s, "float_array")
                if fa is not None and fa.text:
                    arr = np.array(fa.text.split(), dtype=float)
                    acc = s.find(".//{*}accessor")
                    stride = int(acc.get("stride", 3)) if acc is not None else 3
                    sources[s.get("id")] = arr.reshape(-1, stride)[:, :3]
            verts = _child(mesh, "vertices")
            vid, vpos = None, None
            if verts is not None:
                vid = verts.get("id")
                for inp in _children(verts, "input"):
                    if inp.get("semantic") == "POSITION":
                        vpos = sources.get(inp.get("source").lstrip("#"))
            tris = []
            for prim in mesh:
                k = _tag(prim)
                if k not in ("triangles", "polylist"):
                    continue
                inputs = _children(prim, "input")
                n_off = max(int(i.get("offset", 0)) for i in inputs) + 1
                pos, off = None, 0
                for i in inputs:
                    if i.get("semantic") == "VERTEX" and i.get("source").lstrip("#") == vid:
                        pos, off = vpos, int(i.get("offset", 0))
                    elif i.get("semantic") == "POSITION":
                        pos, off = sources.get(i.get("source").lstrip("#")), int(i.get("offset", 0))
                p_el = _child(prim, "p")
                if pos is None or p_el is None or not p_el.text:
                    continue
                idx = np.array(p_el.text.split(), dtype=np.int64).reshape(-1, n_off)[:, off]
                if k == "triangles":
                    tris.append(pos[idx.reshape(-1, 3)])
                else:
                    vc = np.array(_child(prim, "vcount").text.split(), dtype=int)
                    start = 0
                    for c in vc:
                        poly = idx[start:start + c]
                        start += c
                        for j in range(1, c - 1):
                            tris.append(pos[[poly[0], poly[j], poly[j + 1]]][None])
            if tris:
                geoms[g.get("id")] = np.concatenate(tris)

    out = []

    def _match(node, gid):
        if submesh is None:
            return True
        names = {node.get("id"), node.get("name"), gid}
        if gid and gid.endswith("-mesh"):
            names.add(gid[:-len("-mesh")])
        return submesh in names

    def walk(node, M):
        M = M @ _dae_node_matrix(node)
        for c in node:
            k = _tag(c)
            if k == "instance_geometry":
                gid = c.get("url", "").lstrip("#")
                g = geoms.get(gid)
                if g is not None and _match(node, gid):
                    out.append(g @ M[:3, :3].T + M[:3, 3])
            elif k == "node":
                walk(c, M)

    for vs in root.iter():
        if _tag(vs) == "visual_scene":
            for n in _children(vs, "node"):
                walk(n, np.eye(4))
    if not out:
        return np.zeros((0, 3, 3))
    tris = np.concatenate(out) * unit
    if up == "Y_UP":  # Y arriba -> Z arriba (rotacion +90 en X)
        tris = tris @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]).T
    elif up == "X_UP":
        tris = tris @ np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]).T
    return tris


def load_mesh(path, scale=(1.0, 1.0, 1.0), submesh=None, center=False):
    """submesh/center replican <submesh><name>/<center> de SDF: solo ese trozo
    y, con center, trasladado para que el centro de su AABB quede en el
    origen (gz::common::SubMesh::Center)."""
    key = (path, tuple(scale), submesh, center)
    if key not in _MESH_CACHE:
        ext = os.path.splitext(path)[1].lower()
        if ext == ".stl":
            if submesh is not None:
                raise ValueError(f"submesh en un STL: {path}")
            tris = load_stl(path)
        elif ext == ".dae":
            tris = load_dae(path, submesh)
            if submesh is not None and len(tris) == 0:
                raise ValueError(f"submesh '{submesh}' no encontrado en {path}")
        else:
            raise ValueError(f"formato de malla no soportado: {path}")
        if center and len(tris):
            pts = tris.reshape(-1, 3)
            tris = tris - (pts.min(0) + pts.max(0)) / 2.0
        _MESH_CACHE[key] = tris * np.array(scale)
    return _MESH_CACHE[key]


# ---- Mundo -----------------------------------------------------------------

@dataclass
class Geom:
    model: str
    kind: str        # 'collision' | 'visual'
    shape: str       # 'box' | 'cylinder' | 'sphere' | 'plane' | 'mesh'
    tris: np.ndarray  # (N,3,3) frame mundo
    static: bool = True
    source: str = ""


@dataclass
class Actor:
    name: str
    pose: np.ndarray                      # 4x4 del <include>/<actor>
    waypoints: list = field(default_factory=list)  # [(t, 4x4)]


@dataclass
class World:
    name: str
    path: str
    geoms: list
    actors: list
    unresolved: list   # URIs/mallas que no se pudieron cargar (avisos)
    models: dict       # nombre -> {'pose':4x4, 'static':bool, 'source':str}


def _resolve(uri, base_dir, models_dirs):
    if uri.startswith("model://"):
        rel = uri[len("model://"):]
        for d in models_dirs:
            p = os.path.join(d, rel)
            if os.path.exists(p):
                return p
        return None
    if uri.startswith(("http://", "https://", "file://")):
        return uri[len("file://"):] if uri.startswith("file://") else None
    p = os.path.join(base_dir, uri)
    return p if os.path.exists(p) else None


def _geometry(gel, base_dir, models_dirs, unresolved):
    if gel is None or len(gel) == 0:
        return None, None
    g = gel[0]
    k = _tag(g)
    if k == "box":
        return k, box_tris(*[float(v) for v in _text(g, "size").split()])
    if k == "cylinder":
        return k, cylinder_tris(float(_text(g, "radius")), float(_text(g, "length")))
    if k == "sphere":
        return k, sphere_tris(float(_text(g, "radius")))
    if k == "plane":
        n = [float(v) for v in (_text(g, "normal") or "0 0 1").split()]
        s = [float(v) for v in (_text(g, "size") or "100 100").split()]
        return k, plane_tris(n, s)
    if k == "mesh":
        uri = _text(g, "uri")
        path = _resolve(uri, base_dir, models_dirs)
        if path is None:
            unresolved.append(uri)
            return None, None
        scale = [float(v) for v in (_text(g, "scale") or "1 1 1").split()]
        sm = _child(g, "submesh")
        name = _text(sm, "name") if sm is not None else None
        center = sm is not None and (_text(sm, "center", "false") or "").lower() in ("true", "1")
        return k, load_mesh(path, scale, name, center)
    unresolved.append(f"geometria <{k}> no soportada")
    return None, None


def _walk_model(mel, T_model, name, base_dir, models_dirs, geoms, unresolved):
    static = (_text(mel, "static", "false") or "").lower() in ("true", "1")
    for link in _children(mel, "link"):
        T_link = T_model @ pose_matrix(_text(link, "pose"))
        for kind in ("collision", "visual"):
            for c in _children(link, kind):
                shape, tris = _geometry(_child(c, "geometry"), base_dir,
                                        models_dirs, unresolved)
                if tris is None:
                    continue
                T = T_link @ pose_matrix(_text(c, "pose"))
                geoms.append(Geom(name, kind, shape, transform(tris, T), static,
                                  f"{name}/{link.get('name')}/{c.get('name')}"))
    return static


def _parse_actor(ael, T, name):
    a = Actor(name, T)
    for traj in ael.iter():
        if _tag(traj) != "trajectory":
            continue
        for wp in _children(traj, "waypoint"):
            a.waypoints.append((float(_text(wp, "time", "0")),
                                pose_matrix(_text(wp, "pose"))))
    return a


def load_world(path, models_dirs):
    root = read_xml(path)
    wel = root if _tag(root) == "world" else _child(root, "world")
    base_dir = os.path.dirname(path)
    geoms, actors, unresolved, models = [], [], [], {}
    for el in wel:
        k = _tag(el)
        if k == "model":
            T = pose_matrix(_text(el, "pose"))
            st = _walk_model(el, T, el.get("name"), base_dir, models_dirs, geoms, unresolved)
            models[el.get("name")] = {"pose": T, "static": st, "source": "inline"}
        elif k == "actor":
            actors.append(_parse_actor(el, pose_matrix(_text(el, "pose")), el.get("name")))
        elif k == "include":
            uri = _text(el, "uri")
            mdir = _resolve(uri, base_dir, models_dirs)
            if mdir is None:
                unresolved.append(uri)
                continue
            sdf = os.path.join(mdir, "model.sdf")
            cfg = os.path.join(mdir, "model.config")
            if os.path.exists(cfg):
                c = read_xml(cfg)
                s = c.find(".//{*}sdf") if c.find(".//{*}sdf") is not None else c.find(".//sdf")
                if s is not None and s.text:
                    sdf = os.path.join(mdir, s.text.strip())
            mroot = read_xml(sdf)
            inner = _child(mroot, "model")
            if inner is None:
                inner = _child(mroot, "actor")
            if inner is None:
                unresolved.append(f"{uri} (sin <model>/<actor>)")
                continue
            name = _text(el, "name") or inner.get("name")
            # La <pose> del <include> SUSTITUYE a la del modelo (semantica SDF).
            T = pose_matrix(_text(el, "pose")) if _child(el, "pose") is not None \
                else pose_matrix(_text(inner, "pose"))
            if _tag(inner) == "actor":
                actors.append(_parse_actor(inner, T, name))
                continue
            st = _walk_model(inner, T, name, mdir, models_dirs, geoms, unresolved)
            if (_text(el, "static", "") or "").lower() in ("true", "1"):
                st = True
            models[name] = {"pose": T, "static": st, "source": uri}
    return World(wel.get("name"), path, geoms, actors, unresolved, models)
