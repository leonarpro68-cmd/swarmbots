
"""Prueba offline del arbitraje de turnos de depósito, sin Gazebo."""

import rclpy
from swarm_behavior.central_planner import CentralPlanner


def test_deposit_turns():
    rclpy.init()
    p = None

    try:
        p = CentralPlanner()
        p.n_robots = 2
        p.robots = ["summit0", "summit1"]

        deposits = {"deposit_0_verde": (0.0, 0.0)}

        # Caso 1: dos robots hacia el mismo depósito.
        p.state = {"summit0": "deliver", "summit1": "deliver"}
        p.target = {
            "summit0": ("deposit", "deposit_0_verde"),
            "summit1": ("deposit", "deposit_0_verde"),
        }
        p.carried = {
            "summit0": "trash_0_verde",
            "summit1": "trash_1_verde",
        }
        p.robot_xy = {
            "summit0": (3.0, 0.0),
            "summit1": (1.0, 0.0),
        }
        p.deposit_holder = {}
        p.deposit_lock_t = {}

        p._update_deposit_turns(deposits)

        assert len(p.deposit_holder) == 1
        assert p.deposit_holder["deposit_0_verde"] == "summit1"

        # Caso 2: el turno no cambia por cercanía.
        p.robot_xy["summit0"] = (0.2, 0.0)
        p._update_deposit_turns(deposits)

        assert p.deposit_holder["deposit_0_verde"] == "summit1"

        # Caso 3: prioridad del robot que ya está soltando.
        p.deposit_holder = {}
        p.deposit_lock_t = {}
        p.state = {"summit0": "release", "summit1": "deliver"}
        p.robot_xy = {
            "summit0": (3.0, 0.0),
            "summit1": (0.5, 0.0),
        }

        p._update_deposit_turns(deposits)

        assert p.deposit_holder["deposit_0_verde"] == "summit0"

        # Caso 4: el turno pasa al siguiente robot.
        p.state["summit0"] = "seek"
        p.target["summit0"] = None
        p.carried["summit0"] = None

        p._update_deposit_turns(deposits)

        assert p.deposit_holder["deposit_0_verde"] == "summit1"

        # Caso 5: caducidad del turno.
        p.state = {"summit0": "deliver", "summit1": "deliver"}
        p.target["summit0"] = ("deposit", "deposit_0_verde")
        p.deposit_lock_t["deposit_0_verde"] -= (
            p.deposit_lock_timeout + 1.0
        )

        p._update_deposit_turns(deposits)

        assert p.deposit_holder["deposit_0_verde"] == "summit0"

        # Caso 6: depósitos distintos.
        dep2 = {
            "deposit_0_verde": (0.0, 0.0),
            "deposit_1_azul": (10.0, 0.0),
        }

        p.deposit_holder = {}
        p.deposit_lock_t = {}
        p.state = {"summit0": "deliver", "summit1": "deliver"}
        p.target = {
            "summit0": ("deposit", "deposit_0_verde"),
            "summit1": ("deposit", "deposit_1_azul"),
        }

        p._update_deposit_turns(dep2)

        assert p.deposit_holder == {
            "deposit_0_verde": "summit0",
            "deposit_1_azul": "summit1",
        }

        # Caso 7: verificar el estado al desactivar turnos.
        p.deposit_turns = False

        assert p.deposit_holder.get("deposit_0_verde") == "summit0"

    finally:
        if p is not None:
            p.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    test_deposit_turns()
    print("RESULTADO: TODO OK")

