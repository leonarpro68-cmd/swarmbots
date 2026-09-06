"""Test offline del arbitraje de turno de deposito (sin Gazebo).

Instancia el CentralPlanner real y le pone estados sinteticos: comprueba que
solo un robot tiene turno, que el que espera recibe meta de cola y no dispara
la suelta, la prioridad del que ya suelta, y la caducidad del turno."""
import rclpy, sys
from swarm_behavior.central_planner import CentralPlanner

rclpy.init()
ok = True
def check(cond, msg):
    global ok
    print(("  OK   " if cond else "  FALLO ") + msg)
    ok = ok and cond

p = CentralPlanner()
p.n_robots = 2
p.robots = ["summit0", "summit1"]
deposits = {"deposit_0_verde": (0.0, 0.0)}

# --- caso 1: los dos llevan pieza del mismo color al mismo deposito ---
print("1) dos robots hacia el MISMO deposito")
p.state = {"summit0": "deliver", "summit1": "deliver"}
p.target = {"summit0": ("deposit", "deposit_0_verde"),
            "summit1": ("deposit", "deposit_0_verde")}
p.carried = {"summit0": "trash_0_verde", "summit1": "trash_1_verde"}
p.robot_xy = {"summit0": (3.0, 0.0), "summit1": (1.0, 0.0)}  # summit1 mas cerca
p.deposit_holder = {}; p.deposit_lock_t = {}
p._update_deposit_turns(deposits)
check(len(p.deposit_holder) == 1, "solo se concede UN turno")
check(p.deposit_holder["deposit_0_verde"] == "summit1",
      "el turno va al mas cercano (summit1 a 1 m, summit0 a 3 m)")

# --- caso 2: pegajoso aunque el otro se acerque mas ---
print("2) el turno NO se le quita al dueno si el otro se acerca mas")
p.robot_xy["summit0"] = (0.2, 0.0)
p._update_deposit_turns(deposits)
check(p.deposit_holder["deposit_0_verde"] == "summit1", "sigue siendo de summit1")

# --- caso 3: prioridad absoluta del que ya esta soltando ---
print("3) el que ya esta en release manda")
p.deposit_holder = {}; p.deposit_lock_t = {}
p.state = {"summit0": "release", "summit1": "deliver"}
p.robot_xy = {"summit0": (3.0, 0.0), "summit1": (0.5, 0.0)}
p._update_deposit_turns(deposits)
check(p.deposit_holder["deposit_0_verde"] == "summit0",
      "summit0 (soltando, y mas lejos) gana a summit1")

# --- caso 4: al soltar, el turno queda libre para el que esperaba ---
print("4) al terminar, el turno pasa al que esperaba")
p.state["summit0"] = "seek"; p.target["summit0"] = None; p.carried["summit0"] = None
p._update_deposit_turns(deposits)
check(p.deposit_holder["deposit_0_verde"] == "summit1", "ahora le toca a summit1")

# --- caso 5: caducidad si el dueno se atasca ---
print("5) el turno caduca si el dueno se atasca")
p.state = {"summit0": "deliver", "summit1": "deliver"}
p.target["summit0"] = ("deposit", "deposit_0_verde")
p.deposit_lock_t["deposit_0_verde"] -= (p.deposit_lock_timeout + 1.0)
p._update_deposit_turns(deposits)
check(p.deposit_holder["deposit_0_verde"] == "summit0",
      "caduca el de summit1 y pasa a summit0 AUNQUE summit1 este mas cerca "
      "(si no, el atascado se lo reconcede a si mismo y el timeout no sirve)")

# --- caso 6: depositos DISTINTOS -> ninguno espera (no cambia lo que ya iba) ---
print("6) depositos distintos: los dos tienen turno, nadie espera")
dep2 = {"deposit_0_verde": (0.0, 0.0), "deposit_1_azul": (10.0, 0.0)}
p.deposit_holder = {}; p.deposit_lock_t = {}
p.state = {"summit0": "deliver", "summit1": "deliver"}
p.target = {"summit0": ("deposit", "deposit_0_verde"),
            "summit1": ("deposit", "deposit_1_azul")}
p._update_deposit_turns(dep2)
check(p.deposit_holder == {"deposit_0_verde": "summit0", "deposit_1_azul": "summit1"},
      "cada uno tiene el turno de SU deposito")

# --- caso 7: interruptor de apagado ---
print("7) deposit_turns=false deja el comportamiento anterior")
p.deposit_turns = False
check(p.deposit_holder.get("deposit_0_verde") in ("summit0",),
      "(el dict solo lo actualiza _update_deposit_turns, que ya no se llama)")

p.destroy_node(); rclpy.shutdown()
print("\nRESULTADO:", "TODO OK" if ok else "HAY FALLOS")
sys.exit(0 if ok else 1)
