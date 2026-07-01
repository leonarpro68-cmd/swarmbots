#!/usr/bin/env python3
"""
bag_to_act_hdf5.py — Convierte un rosbag2 de teleop (DEMO 4) en un episodio
HDF5 con el formato que espera ACT (Action Chunking Transformer, Zhao et al.
2023 / ALOHA) y que `lerobot` también sabe ingerir. Pensado para entrenar UNA
política compartida desplegada en N robots idénticos del enjambre.

Estructura de salida (un archivo = un episodio). Las dimensiones dependen de
los flags --include_xy y --holonomic (ver abajo):

    sim                       (atributo raiz, True)
    /observations/qpos        (T, 2|4) por defecto [yaw, gripper_width]
                                        con --include_xy: [x, y, yaw, gripper_width]
    /observations/qvel        (T, 3)   [vx, vy, wz]  (twist del odom, frame cuerpo)
    /observations/imu         (T, 10)  [qx,qy,qz,qw, wx,wy,wz, ax,ay,az]
    /observations/images/<cam>(T, H, W, 3) uint8 RGB   (front, overhead)
    /action                   (T, 3|4) holonomico (def) [cmd_vx, cmd_vy, cmd_wz, grasp]
                                        --no-holonomic:  [cmd_vx, cmd_wz, grasp]
                                        grasp = estado LATCHED 0/1 (1 = agarrar/sostener)

DECISIONES DE OBSERVACION/ACCION (ver flags):
  * qpos EGOCENTRICO por defecto (--include_xy=False): x,y del odom por encoders
    DERIVAN y son absolutos al mundo -> frágiles para una política COMPARTIDA
    entre N robots (cada uno arranca en un (x,y) distinto). La política se apoya
    en la CAMARA para localizar la pieza, no en la odometría global. yaw e IMU
    (orientación/giro) SIEMPRE se incluyen (son relativos, no derivan en x,y).
    Usa --include_xy solo si entrenas algo dependiente de la pose mundial.
  * Acción HOLONOMICA por defecto (--holonomic=True): este robot es un Summit
    XLS con ruedas MECANUM (plugin MecanumDrive) y la teleop PS5 comanda
    linear.y (strafe) -> cmd_vy es un GRADO DE LIBERTAD REAL, no ~0. Por eso NO
    se elimina por defecto. Para un robot skid-steer/diferencial (sin strafe)
    pasa --no-holonomic y se colapsa a [vx, wz, grasp]. El conversor MIDE cmd_vy
    en el bag y avisa si el flag no cuadra con los datos.

FRECUENCIA (--hz, def 15 = tasa de las cámaras): re-muestreo a frecuencia FIJA
con retención de orden cero (ZOH: en cada instante, el último mensaje recibido
de cada tópico). OJO: el instante de AGARRE (cierre de pinza) es un evento de
ALTA FRECUENCIA; 15 Hz puede quedarse corto para capturarlo bien. Si al desplegar
la política el cierre de pinza sale errático o se pierde el momento del grasp,
regraba/reconvierte a --hz 30 (a costa de episodios ~2x más pesados).

Marcas de tiempo: las de RECEPCION del bag (no las del header), uniformes para
todos los tópicos e independientes de sim-time (los Empty de grasp/release no
tienen header). El timeline se acota al solape cámara∩odom (no extrapola frames).

Uso tipico:
    python3 scripts/bag_to_act_hdf5.py demo_143052 -o episode_0.hdf5
    python3 scripts/bag_to_act_hdf5.py demo_143052 -o ep.hdf5 --debug_dump_frames 5
    # lote:
    for d in demo_*; do python3 scripts/bag_to_act_hdf5.py "$d" -o "${d}.hdf5"; done

Requiere: rosbag2_py (ROS 2 Jazzy), numpy, h5py. Para CompressedImage/dump: cv2, Pillow.
    pip install h5py           # o: sudo apt install python3-h5py
"""

import argparse
import math
import os
import sys

import numpy as np

VY_EPS = 1e-3  # umbral para considerar que hay strafe real en el bag


# ---------------------------------------------------------------------------
# Lectura del bag
# ---------------------------------------------------------------------------
def read_bag(bag_path, storage_id=""):
    """Devuelve {topic: (type_str, [(t_ns, msg), ...])} leyendo TODO el bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    typemap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msgclass = {}
    out = {name: (typ, []) for name, typ in typemap.items()}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        typ = typemap[topic]
        if typ not in msgclass:
            msgclass[typ] = get_message(typ)
        msg = deserialize_message(data, msgclass[typ])
        out[topic][1].append((t_ns, msg))
    return out


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def decode_raw_image(msg):
    """sensor_msgs/Image -> np.uint8 (H, W, 3) RGB, respetando el row step.

    El bridge gz->ros de una cámara con <format>R8G8B8</format> entrega encoding
    'rgb8' (ya RGB, sin swap). Si la fuente fuese bgr*, se invierten los canales.
    """
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        ch = 3
    elif enc in ("rgba8", "bgra8"):
        ch = 4
    elif enc in ("mono8", "8uc1"):
        ch = 1
    else:
        raise ValueError(f"encoding de imagen no soportado: {msg.encoding}")
    step = msg.step if msg.step else w * ch
    rows = buf[: step * h].reshape(h, step)
    img = rows[:, : w * ch].reshape(h, w, ch)
    if ch == 1:
        img = np.repeat(img, 3, axis=2)
    elif ch == 4:
        img = img[:, :, :3]
    if enc.startswith("bgr"):
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img)


def decode_compressed_image(msg):
    """sensor_msgs/CompressedImage (jpeg/png) -> np.uint8 (H, W, 3) RGB.

    cv2.imdecode devuelve SIEMPRE en orden BGR -> se invierte a RGB.
    """
    import cv2

    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)  # OpenCV decodifica a BGR
    if bgr is None:
        raise ValueError(f"no se pudo decodificar CompressedImage (format='{msg.format}')")
    return np.ascontiguousarray(bgr[:, :, ::-1])  # BGR -> RGB


def decode_image(type_str, msg):
    """Despacha según el tipo de mensaje REAL del tópico (no se asume)."""
    if type_str.endswith("CompressedImage"):
        return decode_compressed_image(msg)
    if type_str.endswith("Image"):
        return decode_raw_image(msg)
    raise ValueError(f"tipo de imagen no soportado: {type_str}")


def zoh_index(times, t):
    """Indice del ultimo elemento de `times` (ordenado) con times[i] <= t, o -1."""
    import bisect

    return bisect.bisect_right(times, t) - 1


# ---------------------------------------------------------------------------
# Conversion
# ---------------------------------------------------------------------------
def convert(args):
    try:
        import h5py
    except ImportError:
        sys.exit("ERROR: falta h5py.  pip install h5py  (o sudo apt install python3-h5py)")

    ns = args.robot
    topics = {
        "odom": f"/{ns}/odom",
        "imu": f"/{ns}/imu",
        "joints": f"/{ns}/joint_states",
        "cmd": f"/{ns}/cmd_vel",
        "grasp": f"/{ns}/gripper/grasp",
        "release": f"/{ns}/gripper/release",
    }
    cam_topics = {"front": f"/{ns}/camera", "overhead": args.overhead_topic}

    print(f"Leyendo bag: {args.bag}")
    data = read_bag(args.bag, args.storage)
    for k, v in data.items():
        if v[1]:
            print(f"  {k:40s} {v[0]:32s} {len(v[1]):6d} msgs")

    def series(topic):
        if topic not in data or not data[topic][1]:
            return [], []
        return [t for t, _ in data[topic][1]], [m for _, m in data[topic][1]]

    odom_t, odom_m = series(topics["odom"])
    imu_t, imu_m = series(topics["imu"])
    jnt_t, jnt_m = series(topics["joints"])
    cmd_t, cmd_m = series(topics["cmd"])
    grasp_t, _ = series(topics["grasp"])
    release_t, _ = series(topics["release"])

    if not odom_t:
        sys.exit(f"ERROR: el bag no contiene {topics['odom']} (necesario para qpos/timeline)")

    # --- FIX 1: comprobar cmd_vy REAL en el bag vs el flag --holonomic ------
    max_cmd_vy = max((abs(c.linear.y) for c in cmd_m), default=0.0)
    if args.holonomic and max_cmd_vy < VY_EPS:
        print(f"  [AVISO] --holonomic activo pero cmd_vy es ~0 en el bag "
              f"(max|vy|={max_cmd_vy:.4f}). Si el robot es skid-steer, usa "
              f"--no-holonomic para quitar la columna vy muerta.")
    if not args.holonomic and max_cmd_vy >= VY_EPS:
        print(f"  [AVISO] --no-holonomic pero cmd_vy tiene valores REALES "
              f"(max|vy|={max_cmd_vy:.4f}). ¡Estás descartando strafe usado! "
              f"Este robot es MECANUM; probablemente quieras --holonomic.")

    cam_t, cam_m, cam_type = {}, {}, {}
    for name, tp in cam_topics.items():
        ct, cm = series(tp)
        if ct:
            cam_t[name], cam_m[name], cam_type[name] = ct, cm, data[tp][0]
        else:
            print(f"  (aviso) sin imagenes en {tp} -> cámara '{name}' omitida")

    # --- Timeline a frecuencia fija, acotado al solape cámara∩odom ---------
    starts = [odom_t[0]] + [v[0] for v in cam_t.values()]
    ends = [odom_t[-1]] + [v[-1] for v in cam_t.values()]
    t0, t1 = max(starts), min(ends)
    if t1 <= t0:
        sys.exit("ERROR: las series de cámara/odom no se solapan en el tiempo")
    dt = 1.0 / args.hz
    n = int((t1 - t0) / 1e9 / dt) + 1
    timeline = [t0 + int(round(k * dt * 1e9)) for k in range(n)]
    print(f"Episodio: {n} frames @ {args.hz} Hz  ({(t1 - t0) / 1e9:.1f} s)  "
          f"| holonomic={args.holonomic} include_xy={args.include_xy}")

    # --- Eventos de agarre -> estado latcheado 0/1 ------------------------
    events = sorted([(t, 1) for t in grasp_t] + [(t, 0) for t in release_t])
    ev_t = [e[0] for e in events]
    ev_v = [e[1] for e in events]

    def finger_width(msg):
        try:
            i = msg.name.index(args.finger_joint)
            return float(msg.position[i])
        except (ValueError, IndexError):
            return 0.0

    # --- Dimensiones/etiquetas según flags --------------------------------
    if args.include_xy:
        qpos_labels = ["x", "y", "yaw", "gripper_width"]
        yaw_col, fin_col = 2, 3
    else:
        qpos_labels = ["yaw", "gripper_width"]
        yaw_col, fin_col = 0, 1
    if args.holonomic:
        action_labels = ["cmd_vx", "cmd_vy", "cmd_wz", "grasp"]
    else:
        action_labels = ["cmd_vx", "cmd_wz", "grasp"]

    qpos = np.zeros((n, len(qpos_labels)), np.float32)
    qvel = np.zeros((n, 3), np.float32)
    imu = np.zeros((n, 10), np.float32)
    action = np.zeros((n, len(action_labels)), np.float32)

    # Pre-alocar imágenes decodificando el primer frame de cada cámara
    imgs = {}
    for name in cam_t:
        first = decode_image(cam_type[name], cam_m[name][0])
        imgs[name] = np.zeros((n,) + first.shape, np.uint8)

    for k, t in enumerate(timeline):
        io = zoh_index(odom_t, t)
        if io >= 0:
            o = odom_m[io]
            if args.include_xy:
                p = o.pose.pose.position
                qpos[k, 0], qpos[k, 1] = p.x, p.y
            qpos[k, yaw_col] = yaw_from_quat(o.pose.pose.orientation)
            tw = o.twist.twist
            qvel[k] = (tw.linear.x, tw.linear.y, tw.angular.z)
        ij = zoh_index(jnt_t, t)
        if ij >= 0:
            qpos[k, fin_col] = finger_width(jnt_m[ij])
        ii = zoh_index(imu_t, t)
        if ii >= 0:
            m = imu_m[ii]
            q, w_, a = m.orientation, m.angular_velocity, m.linear_acceleration
            imu[k] = (q.x, q.y, q.z, q.w, w_.x, w_.y, w_.z, a.x, a.y, a.z)
        ic = zoh_index(cmd_t, t)
        if ic >= 0:
            c = cmd_m[ic]
            if args.holonomic:
                action[k, 0], action[k, 1], action[k, 2] = c.linear.x, c.linear.y, c.angular.z
            else:
                action[k, 0], action[k, 1] = c.linear.x, c.angular.z
        ie = zoh_index(ev_t, t)
        action[k, -1] = ev_v[ie] if ie >= 0 else 0.0
        for name in cam_t:
            ix = zoh_index(cam_t[name], t)
            imgs[name][k] = decode_image(cam_type[name], cam_m[name][max(ix, 0)])

    # --- FIX 4: volcado de frames PNG para inspección visual ---------------
    if args.debug_dump_frames > 0 and imgs:
        _dump_frames(args, imgs)

    # --- Escritura HDF5 ---------------------------------------------------
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with h5py.File(args.out, "w") as f:
        f.attrs["sim"] = True
        f.attrs["rate_hz"] = args.hz
        f.attrs["robot"] = ns
        f.attrs["holonomic"] = args.holonomic
        f.attrs["include_xy"] = args.include_xy
        f.attrs["qpos_labels"] = ",".join(qpos_labels)
        f.attrs["action_labels"] = ",".join(action_labels)
        obs = f.create_group("observations")
        obs.create_dataset("qpos", data=qpos)
        obs.create_dataset("qvel", data=qvel)
        obs.create_dataset("imu", data=imu)
        img_g = obs.create_group("images")
        for name, arr in imgs.items():
            img_g.create_dataset(name, data=arr, dtype="uint8",
                                 chunks=(1,) + arr.shape[1:], compression="gzip",
                                 compression_opts=4)
        f.create_dataset("action", data=action)

    grasped = int((action[:, -1] > 0.5).sum())
    print(f"OK -> {args.out}")
    print(f"  qpos {qpos.shape} [{','.join(qpos_labels)}]  "
          f"action {action.shape} [{','.join(action_labels)}]")
    print(f"  cams {[f'{nm}:{imgs[nm].shape[1:]}' for nm in imgs]}")
    print(f"  frames con agarre activo: {grasped}/{n}  |  max|cmd_vy|={max_cmd_vy:.4f}")


def _dump_frames(args, imgs):
    """Guarda hasta N frames por cámara como PNG para verificar color/contenido."""
    from PIL import Image as PILImage

    ddir = os.path.join(os.path.dirname(os.path.abspath(args.out)), "debug_frames")
    os.makedirs(ddir, exist_ok=True)
    total = 0
    for name, arr in imgs.items():
        cnt = min(args.debug_dump_frames, arr.shape[0])
        idxs = np.unique(np.linspace(0, arr.shape[0] - 1, cnt).astype(int))
        for fi in idxs:
            PILImage.fromarray(arr[fi]).save(os.path.join(ddir, f"{name}_frame{fi:04d}.png"))
            total += 1
    print(f"  [debug] volcados {total} PNG (RGB) en {ddir}  -> inspecciona colores a ojo")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="directorio del rosbag2 (el que contiene metadata.yaml)")
    ap.add_argument("-o", "--out", required=True, help="ruta del HDF5 de salida")
    ap.add_argument("--hz", "-r", "--rate", dest="hz", type=float, default=15.0,
                    help="frecuencia de re-muestreo en Hz (def 15 = tasa de las cámaras). "
                         "El grasp es de alta frecuencia; prueba 30 si el cierre sale errático.")
    ap.add_argument("--holonomic", default=True, action=argparse.BooleanOptionalAction,
                    help="mantener cmd_vy (strafe mecanum) en la acción. Def True (robot "
                         "MECANUM). --no-holonomic para skid-steer: acción = [vx, wz, grasp].")
    ap.add_argument("--include_xy", default=False, action=argparse.BooleanOptionalAction,
                    help="incluir x,y (odom absoluto) en qpos. Def False (egocéntrico: la "
                         "política compartida usa la cámara, no la odom global). yaw/IMU siempre.")
    ap.add_argument("--debug_dump_frames", type=int, default=0, metavar="N",
                    help="guardar N frames PNG por cámara (RGB) en ./debug_frames para "
                         "inspección visual del color/contenido antes de generar en masa.")
    ap.add_argument("--robot", default="summit0", help="namespace del robot (def summit0)")
    ap.add_argument("--overhead-topic", default="/overhead/image",
                    help="tópico de la cámara cenital (def /overhead/image)")
    ap.add_argument("--finger-joint", default="finger_left_joint",
                    help="joint cuya posición es el ancho del gripper (def finger_left_joint)")
    ap.add_argument("--storage", default="",
                    help="storage_id del bag (vacío = autodetectar: mcap/sqlite3)")
    args = ap.parse_args()
    convert(args)


if __name__ == "__main__":
    main()
