#!/usr/bin/env python3
"""
bag_to_act_hdf5.py — Convierte un rosbag2 de teleop (DEMO 4) en un episodio
HDF5 con el formato que espera ACT (Action Chunking Transformer, Zhao et al.
2023 / ALOHA) y que `lerobot` también sabe ingerir.

Estructura de salida (un archivo = un episodio):

    sim                       (atributo raiz, True)
    /observations/qpos        (T, 4)   [x, y, yaw, gripper_width]
    /observations/qvel        (T, 3)   [vx, vy, wz]  (twist del odom, frame cuerpo)
    /observations/imu         (T, 10)  [qx,qy,qz,qw, wx,wy,wz, ax,ay,az]  (futuro/EKF)
    /observations/images/<cam>(T, H, W, 3) uint8 RGB   (front, overhead)
    /action                   (T, 4)   [cmd_vx, cmd_vy, cmd_wz, grasp]
                                        grasp = estado LATCHED 0/1 (1 = agarrar/sostener)

Todo se re-muestrea a una frecuencia FIJA (--rate, def 15 Hz = la de las
camaras) con retencion de orden cero (ZOH: en cada instante se toma el ultimo
mensaje recibido de cada topico). Las marcas de tiempo son las de RECEPCION del
bag (no las del header), uniformes para todos los topicos e independientes de
si el header trae sim-time o no (los Empty de grasp/release no tienen header).

Uso tipico:
    python3 scripts/bag_to_act_hdf5.py demo_143052 -o episode_0.hdf5
    # lote:
    for d in demo_*; do python3 scripts/bag_to_act_hdf5.py "$d" -o "${d}.hdf5"; done

Requiere: rosbag2_py (ROS 2 Jazzy), numpy, h5py.
    pip install h5py           # o: sudo apt install python3-h5py
"""

import argparse
import math
import os
import sys

import numpy as np


# ---------------------------------------------------------------------------
# Lectura del bag
# ---------------------------------------------------------------------------
def read_bag(bag_path, storage_id=""):
    """Devuelve {topic: (type_str, [(t_ns, msg), ...])} leyendo TODO el bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    if os.path.isdir(bag_path):
        # rosbag2 graba un directorio con metadata.yaml + ficheros de datos.
        uri = bag_path
    else:
        uri = bag_path

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
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


def image_to_rgb(msg):
    """sensor_msgs/Image -> np.uint8 (H, W, 3) RGB, respetando el row step."""
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


def zoh_index(times, t):
    """Indice del ultimo elemento de `times` (ordenado) con times[i] <= t, o -1."""
    import bisect

    i = bisect.bisect_right(times, t) - 1
    return i


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
            print(f"  {k:40s} {len(v[1]):6d} msgs")

    def series(topic):
        if topic not in data or not data[topic][1]:
            return [], []
        ts = [t for t, _ in data[topic][1]]
        ms = [m for _, m in data[topic][1]]
        return ts, ms

    odom_t, odom_m = series(topics["odom"])
    imu_t, imu_m = series(topics["imu"])
    jnt_t, jnt_m = series(topics["joints"])
    cmd_t, cmd_m = series(topics["cmd"])
    grasp_t, _ = series(topics["grasp"])
    release_t, _ = series(topics["release"])

    if not odom_t:
        sys.exit(f"ERROR: el bag no contiene {topics['odom']} (necesario para qpos/timeline)")

    cam_t, cam_m = {}, {}
    for name, tp in cam_topics.items():
        ct, cm = series(tp)
        if ct:
            cam_t[name], cam_m[name] = ct, cm
        else:
            print(f"  (aviso) sin imagenes en {tp} -> cámara '{name}' omitida")

    # --- Timeline a frecuencia fija. La acotamos a la 1ª/última imagen
    #     disponible para no extrapolar fotogramas (las cámaras limitan).
    starts = [odom_t[0]] + [v[0] for v in cam_t.values()]
    ends = [odom_t[-1]] + [v[-1] for v in cam_t.values()]
    t0, t1 = max(starts), min(ends)
    if t1 <= t0:
        sys.exit("ERROR: las series de cámara/odom no se solapan en el tiempo")
    dt = 1.0 / args.rate
    n = int((t1 - t0) / 1e9 / dt) + 1
    timeline = [t0 + int(round(k * dt * 1e9)) for k in range(n)]
    print(f"Episodio: {n} frames @ {args.rate} Hz  ({(t1 - t0) / 1e9:.1f} s)")

    # --- Eventos de agarre -> estado latcheado 0/1
    events = sorted([(t, 1) for t in grasp_t] + [(t, 0) for t in release_t])
    ev_t = [e[0] for e in events]
    ev_v = [e[1] for e in events]

    def finger_width(msg):
        try:
            i = msg.name.index(args.finger_joint)
            return float(msg.position[i])
        except (ValueError, IndexError):
            return 0.0

    qpos = np.zeros((n, 4), np.float32)
    qvel = np.zeros((n, 3), np.float32)
    imu = np.zeros((n, 10), np.float32)
    action = np.zeros((n, 4), np.float32)
    imgs = {name: np.zeros((n, cam_m[name][0].height, cam_m[name][0].width, 3), np.uint8)
            for name in cam_t}

    for k, t in enumerate(timeline):
        io = zoh_index(odom_t, t)
        if io >= 0:
            o = odom_m[io]
            p = o.pose.pose.position
            qpos[k, 0], qpos[k, 1] = p.x, p.y
            qpos[k, 2] = yaw_from_quat(o.pose.pose.orientation)
            tw = o.twist.twist
            qvel[k] = (tw.linear.x, tw.linear.y, tw.angular.z)
        ij = zoh_index(jnt_t, t)
        if ij >= 0:
            qpos[k, 3] = finger_width(jnt_m[ij])
        ii = zoh_index(imu_t, t)
        if ii >= 0:
            m = imu_m[ii]
            q, w_, a = m.orientation, m.angular_velocity, m.linear_acceleration
            imu[k] = (q.x, q.y, q.z, q.w, w_.x, w_.y, w_.z, a.x, a.y, a.z)
        ic = zoh_index(cmd_t, t)
        if ic >= 0:
            c = cmd_m[ic]
            action[k, 0], action[k, 1], action[k, 2] = c.linear.x, c.linear.y, c.angular.z
        ie = zoh_index(ev_t, t)
        action[k, 3] = ev_v[ie] if ie >= 0 else 0.0
        for name in cam_t:
            ix = zoh_index(cam_t[name], t)
            imgs[name][k] = image_to_rgb(cam_m[name][max(ix, 0)])

    # --- Escritura HDF5
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with h5py.File(args.out, "w") as f:
        f.attrs["sim"] = True
        f.attrs["rate_hz"] = args.rate
        f.attrs["robot"] = ns
        f.attrs["qpos_labels"] = "x,y,yaw,gripper_width"
        f.attrs["action_labels"] = "cmd_vx,cmd_vy,cmd_wz,grasp"
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

    grasped = int((action[:, 3] > 0.5).sum())
    print(f"OK -> {args.out}")
    print(f"  qpos {qpos.shape}  action {action.shape}  "
          f"cams {[f'{n}:{imgs[n].shape[1:]}' for n in imgs]}")
    print(f"  frames con agarre activo: {grasped}/{n}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="directorio del rosbag2 (el que contiene metadata.yaml)")
    ap.add_argument("-o", "--out", required=True, help="ruta del HDF5 de salida")
    ap.add_argument("-r", "--rate", type=float, default=15.0,
                    help="frecuencia de re-muestreo en Hz (def 15 = la de las cámaras)")
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
