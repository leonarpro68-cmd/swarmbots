"""Coordinador de PLANIFICACIÓN no bloqueante; no contiene ROS ni llamadas a Gazebo.

Contrato: el hilo del ejecutor ROS solicita planes con submit_* y consulta poll().
Los callbacks /odom nunca deben esperar a CBS. Confirmar recogida solo después
que el servicio Gazebo haya confirmado la eliminación, fuera de este módulo.
"""
from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor, Future
from dataclasses import dataclass
from typing import Any
from .bridge import PlanningBridge


@dataclass(frozen=True)
class PlanningOutcome:
    kind: str
    value: Any = None
    error: Exception | None = None


class MissionPlanningCoordinator:
    def __init__(self, bridge: PlanningBridge | None = None, *, max_ct=20000, max_time=65):
        self.bridge = bridge if bridge is not None else PlanningBridge()
        self.max_ct = max_ct
        self.max_time = max_time
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='swarm_cbs')
        self._future: Future | None = None
        self._kind: str | None = None
        self._closed = False
        self._carrying: tuple[str, ...] = ()
        self.delivered: list[str] = []

    @property
    def busy(self) -> bool:
        return self._future is not None

    @property
    def pending(self):
        return self.bridge.pending

    def _submit(self, kind, fn, *args):
        if self._closed:
            raise RuntimeError('El coordinador está cerrado')
        if self.busy:
            raise RuntimeError('Existe un CBS en ejecución')
        self._kind = kind
        self._future = self._pool.submit(fn, *args, max_ct=self.max_ct, max_time=self.max_time)

    def submit_pickup(self, starts):
        if self._carrying:
            raise RuntimeError('Hay basuras recogidas aún sin entregar')
        self._submit('pickup', self.bridge.plan_pickup, tuple(starts))

    def confirm_pickups(self, confirmed_names):
        """Debe invocarse tras llegada de todos y confirmación externa en Gazebo."""
        if self.busy:
            raise RuntimeError('No modificar mapa mientras CBS esté trabajando')
        names = tuple(confirmed_names)
        if len(set(names)) != len(names) or any(n not in self.pending for n in names):
            raise ValueError('Confirmaciones de basura inválidas')
        if self._carrying:
            raise RuntimeError('Ya existe carga pendiente de entrega')
        for name in names:
            self.bridge.confirm_collected(name)
        self._carrying = names

    def submit_delivery(self, starts):
        if not self._carrying:
            raise RuntimeError('Primero debe confirmarse la recogida')
        self._submit('delivery', self.bridge.plan_delivery, tuple(starts), self._carrying)

    def confirm_delivery(self):
        """Invocar solamente una vez los robots llegaron a metas de entrega."""
        if self.busy:
            raise RuntimeError('CBS aún en ejecución')
        if not self._carrying:
            raise RuntimeError('No hay carga pendiente')
        self.delivered.extend(self._carrying)
        self._carrying = ()

    @property
    def mission_complete(self):
        return not self.pending and not self._carrying and not self.busy and len(self.delivered) == len(self.bridge.graph.trash)

    def poll(self) -> PlanningOutcome | None:
        """No bloquea: integrar en timer ROS, NO llamar future.result() si no terminó."""
        future = self._future
        if future is None or not future.done():
            return None
        kind = self._kind
        self._future, self._kind = None, None
        try:
            return PlanningOutcome(kind, value=future.result())
        except Exception as exc:
            return PlanningOutcome(kind, error=exc)

    def close(self):
        self._closed = True
        # No bloquear callbacks; primero detener ROS robots y después cerrar.
        self._pool.shutdown(wait=False, cancel_futures=True)
