"""Herramientas comunes de los mundos GENERADOS (gen_*.py).

Un Builder acumula cajas/cilindros: el visual se FUNDE en una sola malla
COLLADA con un material por color (~10 draw calls; con ~1300 visuales sueltos
cada camara pagaba un draw call por pieza y el RTF bajaba de 0.88 a 0.76) y la
colision se queda en primitivas. write() deja:

  worlds/<name>.sdf                     mundo world_demo con los plugins Harmonic
  models/<name>/meshes/<name>.dae       malla visual
  models/<name>/model.config

Regla de color del dataset ACT: NADA de verde/azul/naranja en el decorado (son
los colores de basura y depositos) y suelo que contraste en luminancia con ellas.
Tras generar un mundo: world_maps.py --worlds <name> --write (el launch usa el
mapa para no colocar nada encima de la geometria).
"""
import math
import os

PKG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "src", "swarm_worlds")


class Builder:
    def __init__(self, name):
        self.name = name
        self.mesh = {}   # rgb -> [caras (tuplas de vertices) en frame mundo]
        self.col = []    # colisiones (primitivas, XML)

    def _add(self, rgb, faces):
        self.mesh.setdefault(rgb, []).extend(faces)

    def collision_box(self, name, x, y, z, sx, sy, sz, yaw=0.0):
        self.col.append(f"""        <collision name="{name}_c"><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>""")

    def box(self, name, x, y, z, sx, sy, sz, rgb, collide=False, yaw=0.0):
        c, s_ = math.cos(yaw), math.sin(yaw)
        hx, hy, hz = sx / 2, sy / 2, sz / 2

        def P(a, b, h):
            return (x + c * a - s_ * b, y + s_ * a + c * b, z + h)
        v = [P(a, b, h) for h in (-hz, hz) for b in (-hy, hy) for a in (-hx, hx)]
        # caras CCW vistas desde fuera
        self._add(rgb, [(v[0], v[2], v[3], v[1]), (v[4], v[5], v[7], v[6]),
                        (v[0], v[1], v[5], v[4]), (v[2], v[6], v[7], v[3]),
                        (v[0], v[4], v[6], v[2]), (v[1], v[3], v[7], v[5])])
        if collide:
            self.collision_box(name, x, y, z, sx, sy, sz, yaw)

    def cyl(self, name, x, y, z, r, h, rgb, collide=False, n=24):
        ring = [(x + r * math.cos(2 * math.pi * k / n), y + r * math.sin(2 * math.pi * k / n))
                for k in range(n)]
        z0, z1 = z - h / 2, z + h / 2
        faces = []
        for k in range(n):
            (ax, ay), (bx, by) = ring[k], ring[(k + 1) % n]
            faces.append(((ax, ay, z0), (bx, by, z0), (bx, by, z1), (ax, ay, z1)))
            faces.append(((x, y, z1), (ax, ay, z1), (bx, by, z1)))
            faces.append(((x, y, z0), (bx, by, z0), (ax, ay, z0)))
        self._add(rgb, faces)
        if collide:
            self.col.append(f"""        <collision name="{name}_c"><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose><geometry><cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder></geometry></collision>""")

    def walls(self, hx, hy, t, h, rgb, band_rgb=None, band_h=1.2):
        """Suelo no: solo las 4 paredes (caras interiores en x=+-hx, y=+-hy), con
        zocalo opcional hacia dentro (referencia de horizonte en la frontal)."""
        for n, x, y, sx, sy in (("n", 0, hy + t / 2, 2 * hx + 2 * t, t),
                                ("s", 0, -hy - t / 2, 2 * hx + 2 * t, t),
                                ("e", hx + t / 2, 0, t, 2 * hy),
                                ("w", -hx - t / 2, 0, t, 2 * hy)):
            self.box(f"wall_{n}", x, y, h / 2, sx, sy, h, rgb, collide=True)
            if band_rgb:
                ox = -0.01 if n == "e" else (0.01 if n == "w" else 0)
                oy = -0.01 if n == "n" else (0.01 if n == "s" else 0)
                self.box(f"band_{n}", x + ox, y + oy, band_h / 2, sx, sy, band_h, band_rgb)

    def write_mesh(self):
        """COLLADA con una <geometry> y un material por color (el .mtl de un OBJ no
        lo aplica Gazebo: salia todo blanco). Normales planas por cara."""
        mdir = os.path.join(PKG, "models", self.name)
        os.makedirs(os.path.join(mdir, "meshes"), exist_ok=True)
        lib_fx, lib_mat, lib_geo, nodes = [], [], [], []
        nv = 0
        for i, (rgb, faces) in enumerate(self.mesh.items()):
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
        with open(os.path.join(mdir, "meshes", f"{self.name}.dae"), "w") as f:
            f.write(dae)
        with open(os.path.join(mdir, "model.config"), "w") as f:
            f.write(f'<?xml version="1.0"?>\n<model><name>{self.name}</name><version>1.0</version>'
                    '<sdf version="1.9">model.sdf</sdf><description>Malla visual de '
                    f'worlds/{self.name}.sdf (generada)</description></model>\n')
        return nv

    def write(self, comment, ambient="0.6 0.6 0.6", background="0.62 0.72 0.80",
              sun_diffuse="0.95 0.95 0.92", sun_dir="-0.3 0.2 -1", cam_z=38, cam_fov=1.3,
              extra=""):
        """Escribe malla + mundo. comment va en la cabecera XML (sin '--')."""
        assert "--" not in comment, "un comentario XML no puede contener '--'"
        sdf = f"""<?xml version="1.0"?>
<!--
{comment}
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
      <ambient>{ambient} 1</ambient>
      <background>{background} 1</background>
      <shadows>0</shadows>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>0</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>{sun_diffuse} 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>{sun_dir}</direction>
    </light>

    <model name="{self.name}">
      <static>true</static>
      <link name="link">
{chr(10).join(self.col)}
        <visual name="visual">
          <geometry><mesh><uri>model://{self.name}/meshes/{self.name}.dae</uri></mesh></geometry>
        </visual>
      </link>
    </model>

    <model name="overhead_camera">
      <static>true</static>
      <pose>0 0 {cam_z} 0 1.5707963 0</pose>
      <link name="link">
        <sensor name="overhead" type="camera">
          <topic>/overhead/image</topic>
          <update_rate>15</update_rate>
          <camera>
            <horizontal_fov>{cam_fov}</horizontal_fov>
            <image><width>900</width><height>900</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>80.0</far></clip>
          </camera>
          <always_on>1</always_on>
          <visualize>false</visualize>
        </sensor>
      </link>
    </model>
{extra}  </world>
</sdf>
"""
        nv = self.write_mesh()
        out = os.path.join(PKG, "worlds", f"{self.name}.sdf")
        with open(out, "w") as f:
            f.write(sdf)
        print(f"{out}: {sum(map(len, self.mesh.values()))} caras ({nv} vertices) en "
              f"{len(self.mesh)} materiales, {len(self.col)} colisiones")
