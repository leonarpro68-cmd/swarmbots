import sys
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
sys.path.insert(0, str(Path(__file__).resolve().parent))
from colocar_basuras_depositos import ELEMENTS, validate, model_sdf, build_manifest

class TestScene(unittest.TestCase):
    def test_geometry(self):
        self.assertTrue(validate())
    def test_sdf_valid(self):
        for e in ELEMENTS:
            root = ET.fromstring(model_sdf(e))
            self.assertEqual(root.find('model').get('name'), e.name)
            self.assertEqual(root.findtext('.//static'), 'true')
            self.assertEqual(len(root.findall('.//collision')), 1 if e.kind == 'trash' else 0)
    def test_manifest(self):
        data = build_manifest()['elements']
        self.assertEqual(len(data), 9)
        self.assertEqual(sum(e['blocking'] for e in data), 6)

if __name__ == '__main__':
    unittest.main()
