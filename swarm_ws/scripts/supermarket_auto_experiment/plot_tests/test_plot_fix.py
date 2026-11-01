"""Read-only synthetic tests of route obstacle rendering and timing correction."""
import importlib.util, json, pathlib, sys, tempfile, unittest
import numpy as np
ROOT=pathlib.Path(__file__).resolve().parent.parent if pathlib.Path(__file__).parent.name=='plot_tests' else pathlib.Path(__file__).resolve().parent

def load(name, fake_clock=False):
    if fake_clock:
        import types
        for n in ('swarm_behavior','swarm_behavior.swarm_planning','swarm_behavior.swarm_planning.hybrid_refinement'):
            sys.modules.setdefault(n,types.ModuleType(n))
        m=types.ModuleType('swarm_behavior.swarm_planning.hybrid_refinement.successive_clock')
        m.clock_from_payload=lambda payload: None
        sys.modules[m.__name__]=m
    spec=importlib.util.spec_from_file_location(name, ROOT/(name+'.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

class Tests(unittest.TestCase):
    def test_geometry_rectangles(self):
        from matplotlib import pyplot as plt
        mod=load('plot_routes_with_obstacles')
        geometry=json.loads((ROOT/'supermarket_obstaculos.json').read_text())
        fig,ax=plt.subplots();mod.draw_obstacles(ax,geometry)
        self.assertEqual(len(ax.patches),len(geometry['obstacles']))
        self.assertGreater(len(ax.patches),10)
        plt.close(fig)
    def test_detect_delayed_onset(self):
        p=load('plot_v3_aligned',fake_clock=True)
        t=np.arange(0,40,.05)
        v=np.where((t>=23)&(t<=33),1.0,0.0)
        smooth_t,smooth_v=p.smooth_uniform(t,v)
        onset=p.onset_time(smooth_t,smooth_v,.08,.30)
        self.assertIsNotNone(onset)
        self.assertAlmostEqual(onset,23,delta=.25)
    def test_no_motion_does_not_forge_alignment(self):
        p=load('plot_v3_aligned',fake_clock=True)
        t=np.linspace(0,10,100)
        self.assertIsNone(p.onset_time(t,np.zeros_like(t)))

if __name__=='__main__':unittest.main()
