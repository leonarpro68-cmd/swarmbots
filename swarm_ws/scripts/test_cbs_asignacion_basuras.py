import unittest
import cbs_asignacion_basuras as app
import cbs_grafo_125 as cbs
from grafo_basuras_dinamico import DynamicTrashGraph

class TestAsignacionCBS(unittest.TestCase):
    def setUp(self):
        self.graph=DynamicTrashGraph()

    def test_six_trash_present(self):
        self.assertEqual(len(self.graph.pending),6)

    def test_approach_neighbors_free(self):
        for name in self.graph.pending:
            choices=app.approach_vertices(self.graph,name)
            self.assertTrue(choices)
            self.assertLessEqual(len(choices),8)
            self.assertNotIn(self.graph.trash[name].node,choices)
            self.assertTrue(all(v in self.graph.vertices for v in choices))

    def test_assignment_unique_and_greedy(self):
        assignments=app.assign_nearest(app.DEFAULT_STARTS,self.graph)
        self.assertEqual(len(assignments),3)
        self.assertEqual(len({name for name,_ in assignments}),3)
        self.assertEqual(assignments[1][0],'trash_N2_naranja')

    def test_all_planned_paths_valid(self):
        graph,assignments,goals,solution=app.plan_first_round()
        self.assertEqual(len(solution['paths']),3)
        self.assertIsNone(cbs.first_conflict(solution['paths']))
        for i,path in enumerate(solution['paths']):
            self.assertEqual(path[0],app.DEFAULT_STARTS[i])
            self.assertIn(path[-1],goals[i])
            for x,y in zip(path,path[1:]):
                self.assertTrue(x==y or y in graph.neighbors[x])
            self.assertTrue(all(v in graph.vertices for v in path))
        self.assertEqual(len(graph.pending),6)

    def test_low_level_multi_can_choose_open_goal(self):
        path=app.low_level_multi(self.graph,0,(8,10),((7,11),(9,11)),frozenset())
        self.assertEqual(len(path)-1,1)
        self.assertIn(path[-1],((7,11),(9,11)))

    def test_low_level_multi_respects_vertex_constraint(self):
        opts=((7,11),(9,11))
        p=app.low_level_multi(self.graph,0,(8,10),opts,
           frozenset({cbs.Constraint(0,1,vertex=(7,11))}))
        self.assertEqual(p[-1],(9,11))

    def test_cbs_resolves_two_robot_crossing(self):
        starts=((9,18),(12,18))
        goal_sets=(((12,18),),((9,18),))
        result=app.solve_multi(self.graph,starts,goal_sets,max_ct=5000)
        self.assertGreater(result['expanded'],1)
        self.assertIsNone(cbs.first_conflict(result['paths']))
        self.assertEqual(tuple(p[-1] for p in result['paths']),((12,18),(9,18)))

    def test_bad_start(self):
        with self.assertRaises(ValueError):
            app.plan_first_round(((8,12),(15,25),(8,18)))

if __name__=='__main__':unittest.main()
