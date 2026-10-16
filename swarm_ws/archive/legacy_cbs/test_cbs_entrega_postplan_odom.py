"""Regresion: el CBS puede invalidar la odometria durante su calculo."""
import time
import unittest
from types import SimpleNamespace
from cbs_entrega_depositos_ros import PickupDeliveryExecutor

class Log:
    def __init__(self): self.messages=[]
    def info(self, msg): self.messages.append(msg)
    def error(self, msg): self.messages.append(msg)

class TestPostPlanOdom(unittest.TestCase):
    def setup_demo(self):
        d=PickupDeliveryExecutor.__new__(PickupDeliveryExecutor)
        d.stage='WAIT_DELIVERY_ALIGN_ODOM'
        d.agents=('summit0','summit1','summit2')
        d.delivery_plan_finished_at=time.monotonic()
        d.delivery_odom_wait_started=d.delivery_plan_finished_at
        d.delivery_odom_wait_timeout=10.0
        d.delivery_align_targets={a:(0.,0.) for a in d.agents}
        d.odom={a:(0.,0.,0.,d.delivery_plan_finished_at-1.,None) for a in d.agents}
        d.fresh=lambda: all(time.monotonic()-d.odom[a][3]<1.0 for a in d.agents)
        d.node=SimpleNamespace(get_logger=lambda:Log())
        d.start_phase=lambda phase,targets: setattr(d,'phase',phase)
        d.stop_all=lambda: setattr(d,'stopped',True)
        return d
    def test_waits_for_new_odom_even_if_old_still_fresh(self):
        d=self.setup_demo()
        d.tick()
        self.assertEqual(d.stage,'WAIT_DELIVERY_ALIGN_ODOM')
        self.assertFalse(hasattr(d,'phase'))
    def test_starts_after_all_three_update(self):
        d=self.setup_demo()
        t=time.monotonic()
        for a in d.agents: d.odom[a]=(0.,0.,0.,t,None)
        d.tick()
        self.assertEqual(d.stage,'DELIVERY')
        self.assertEqual(d.phase,'DELIVERY_ALIGN')
    def test_partial_update_still_waits(self):
        d=self.setup_demo()
        d.odom['summit0']=(0.,0.,0.,time.monotonic(),None)
        d.tick()
        self.assertEqual(d.stage,'WAIT_DELIVERY_ALIGN_ODOM')
    def test_timeout_fails(self):
        d=self.setup_demo()
        d.delivery_odom_wait_started-=11
        d.tick()
        self.assertEqual(d.stage,'FAILED')
        self.assertTrue(d.stopped)

if __name__=='__main__': unittest.main()
