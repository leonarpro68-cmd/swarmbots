"""Pruebas offline para la integración de CBS con el ejecutor por pasos."""
import unittest
import cbs_grafo_125 as cbs
import cbs_ros_executor as ex


class IntegrationTests(unittest.TestCase):
    def test_pad_wait(self):
        routes=ex.pad_paths((((1,1),(2,1)),((5,5),(6,5),(7,5))))
        self.assertEqual(routes[0],((1,1),(2,1),(2,1)))
        self.assertEqual(len(routes[0]),len(routes[1]))

    def test_extract_step(self):
        p=ex.pad_paths((((1,1),(2,1)),((5,5),(5,5),(6,5)),((8,8),(8,9))))
        self.assertEqual(ex.extract_step(p,1)['summit1'],(5,5))
        self.assertEqual(ex.extract_step(p,2)['summit0'],(2,1))
        with self.assertRaises(IndexError):ex.extract_step(p,3)

    def test_cbs_uses_same_graph_and_resolves_conflicts(self):
        # Inicios próximos a las odometrías medidas con seed=42.
        starts=((12,16),(15,25),(8,18))
        goals=((14,18),(8,18),(11,22))
        res=cbs.solve(starts,goals,max_ct=1000)
        self.assertGreater(res['expanded'],1)
        padded=ex.pad_paths(res['paths'])
        self.assertIsNone(cbs.first_conflict(padded))
        for i,p in enumerate(padded):
            self.assertEqual(p[0],starts[i]);self.assertEqual(p[-1],goals[i])
            for a,b in zip(p,p[1:]):
                self.assertTrue(a==b or b in res['graph'].neighbors[a])

if __name__=='__main__': unittest.main(verbosity=2)
