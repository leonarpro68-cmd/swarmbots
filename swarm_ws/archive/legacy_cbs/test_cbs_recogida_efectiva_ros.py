import subprocess
import unittest
from unittest.mock import Mock
from grafo_basuras_dinamico import DynamicTrashGraph
from cbs_recogida_efectiva_ros import apply_confirmed_pickups, find_remove_service, remove_gazebo_model


class Tests(unittest.TestCase):
    def test_one_service(self):
        run = Mock(return_value=subprocess.CompletedProcess([], 0, '/world/tugbot_warehouse/remove\n', ''))
        self.assertEqual(find_remove_service(run), '/world/tugbot_warehouse/remove')

    def test_explicit_world(self):
        self.assertEqual(find_remove_service(requested_world='foo'), '/world/foo/remove')

    def test_successful_delete(self):
        run = Mock(return_value=subprocess.CompletedProcess([], 0, 'data: true\n', ''))
        self.assertTrue(remove_gazebo_model('trash_V1_verde', '/world/foo/remove', run))
        self.assertIn('name: "trash_V1_verde" type: MODEL', run.call_args.args[0])

    def test_failed_delete_not_confirmed(self):
        run = Mock(return_value=subprocess.CompletedProcess([], 0, 'data: false\n', ''))
        with self.assertRaises(RuntimeError):
            remove_gazebo_model('trash_V1_verde', '/world/foo/remove', run)

    def test_three_confirmed(self):
        graph = DynamicTrashGraph()
        names = ['trash_V1_verde', 'trash_N1_naranja', 'trash_A1_azul']
        removed, failures = apply_confirmed_pickups(graph, names, '/service', lambda n, s: True)
        self.assertEqual(removed, names)
        self.assertEqual(failures, {})
        self.assertEqual(len(graph.pending), 3)
        for name in names:
            self.assertIn(graph.trash[name].node, graph.vertices)

    def test_partial_failure(self):
        graph = DynamicTrashGraph()
        names = ['trash_V1_verde', 'trash_N1_naranja', 'trash_A1_azul']
        def remover(name, service):
            if name == names[1]:
                raise RuntimeError('rechazado')
            return True
        removed, failures = apply_confirmed_pickups(graph, names, '/service', remover)
        self.assertEqual(len(removed), 2)
        self.assertIn(names[1], failures)
        self.assertIn(names[1], graph.pending)
        self.assertNotIn(graph.trash[names[1]].node, graph.vertices)

    def test_reject_duplicates(self):
        with self.assertRaises(ValueError):
            apply_confirmed_pickups(DynamicTrashGraph(), ['trash_V1_verde'] * 2, '/service')


if __name__ == '__main__':
    unittest.main()
