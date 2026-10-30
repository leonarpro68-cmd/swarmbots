import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

FILE=Path(__file__).resolve().parents[1]/'d96c2b_live_pipeline.py'
spec=importlib.util.spec_from_file_location('pipeline',FILE)
p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)

class PipelineTest(unittest.TestCase):
    def test_fail_closed_without_scenario(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(SystemExit):
                p.main(['--scenario',str(Path(d)/'absent.json'),'--output',str(Path(d)/'out'),'--accept-namespaced-world-odom'])
    def test_fail_closed_existing_output(self):
        with tempfile.TemporaryDirectory() as d:
            scenario=Path(d)/'scenario.json';scenario.write_text('{}')
            with self.assertRaises(SystemExit):
                p.main(['--scenario',str(scenario),'--output',d,'--accept-namespaced-world-odom'])
    def test_later_stage_requires_state(self):
        with tempfile.TemporaryDirectory() as d:
            scenario=Path(d)/'scenario.json';scenario.write_text('{}')
            with self.assertRaises(SystemExit):
                p.main(['--scenario',str(scenario),'--output',str(Path(d)/'new'),'--stage','pickup2','--accept-namespaced-world-odom'])
    def test_source_certification(self):
        self.assertTrue(p.source_valid({'stages':{'pickup1':{'certified':True,'hybrid_v2':{'certified':True}}}},'pickup1'))
        self.assertFalse(p.source_valid({'stages':{'pickup1':{'certified':True,'hybrid_v2':{'certified':False}}}},'pickup1'))
    def test_stage_status(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'rep.json';path.write_text(json.dumps({'stages':{'pickup1':{'status':'REJECT'}}}))
            with self.assertRaises(RuntimeError):p.stage_valid(path,'pickup1','status','PASS')

if __name__=='__main__':unittest.main()
