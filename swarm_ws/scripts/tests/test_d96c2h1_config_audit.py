import json
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import d96c2h1_config_audit as m

XACRO='''<robot><gazebo>
<plugin name="gz::sim::systems::MecanumDrive"><odom_topic>/model/${ns}/mecanum_odom</odom_topic><tf_topic>/model/${ns}/mecanum_tf</tf_topic></plugin>
</gazebo><gazebo><plugin name="gz::sim::systems::OdometryPublisher"><odom_frame>${ns}/odom</odom_frame><robot_base_frame>${ns}/base_footprint</robot_base_frame><odom_topic>/model/${ns}/odometry</odom_topic><tf_topic>/model/${ns}/tf</tf_topic><odom_publish_frequency>50</odom_publish_frequency></plugin></gazebo></robot>'''
LAUNCH='''summit_xl_omni.urdf.xacro
_generate_sdf(ns_str, xacro_path
/model/{ns_str}/odometry@nav_msgs/msg/Odometry
/model/{ns_str}/mecanum_odom@nav_msgs/msg/Odometry
(f"/model/{ns_str}/odometry",     "odom")
(f"/model/{ns_str}/mecanum_odom", "encoder_odom")
'''

def make_sdf(name):
    return f'''<sdf version="1.10"><model name="{name}">
<plugin name="gz::sim::systems::MecanumDrive"><odom_topic>/model/{name}/mecanum_odom</odom_topic><tf_topic>/model/{name}/mecanum_tf</tf_topic></plugin>
<plugin name="gz::sim::systems::OdometryPublisher"><odom_frame>{name}/odom</odom_frame><robot_base_frame>{name}/base_footprint</robot_base_frame><odom_topic>/model/{name}/odometry</odom_topic><tf_topic>/model/{name}/tf</tf_topic><odom_publish_frequency>50</odom_publish_frequency></plugin>
</model></sdf>'''

def dynamic():
    return {'status':'DYNAMIC_WORLD_ODOM_CONSISTENT_OBSERVED_NOT_TF_PROVEN',
            'sim_clock_progressing':True,'gazebo_truncated_samples':0,'capture_warnings':[],
            'robots':{r:{'status':'DYNAMIC_CONSISTENT','near_identity_observed':True,
                         'sufficient_motion':True,'odom_frames_expected':True,
                         'matched_pairs':1000,'max_xy_delta_m':.001,
                         'max_yaw_delta_deg':.01,'max_timestamp_bracket_s':.016}
                      for r in m.ROBOTS}}

class ConfigAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.source=self.root/'summit_omni.xacro';self.source.write_text(XACRO)
        self.launch=self.root/'sim_summit.launch.py';self.launch.write_text(LAUNCH)
        self.sdfdir=self.root/'sdfs';self.sdfdir.mkdir()
        for r in m.ROBOTS:(self.sdfdir/f'{r}.sdf').write_text(make_sdf(r))
        self.dyn=self.root/'dynamic.json';self.dyn.write_text(json.dumps(dynamic()))
        self.args=type('Args',(),{'source_xacro':self.source,'launch':self.launch,
               'sdf_dir':self.sdfdir,'dynamic_report':self.dyn})()

    def test_valid_fixture_passes_no_execution(self):
        x=m.run(self.args)
        self.assertEqual(x['status'],'PASS_CONFIG_AND_DYNAMIC_OBSERVATION_NOT_TF_CERTIFIED')
        self.assertFalse(x['execution_authorized'])
        self.assertFalse(x['coordinate_frame_equivalence_proven'])
        self.assertFalse(x['tf_tree_certified'])
        self.assertEqual(x['publishers_created'],0)

    def test_generated_sdf_missing_rejected(self):
        (self.sdfdir/'summit1.sdf').unlink()
        self.assertEqual(m.run(self.args)['status'],'REJECT_CONFIGURATION_OR_EVIDENCE')

    def test_generated_odom_frame_change_rejected(self):
        p=self.sdfdir/'summit2.sdf';p.write_text(p.read_text().replace('summit2/odom','odom'))
        self.assertFalse(m.run(self.args)['generated_sdf']['summit2']['passed'])

    def test_odometry_topic_collision_rejected(self):
        p=self.sdfdir/'summit0.sdf';p.write_text(p.read_text().replace('summit0/mecanum_odom','summit0/odometry'))
        self.assertFalse(m.run(self.args)['generated_sdf']['summit0']['passed'])

    def test_missing_odometry_publisher_rejected(self):
        p=self.sdfdir/'summit0.sdf'
        tree=ET.parse(p);root=tree.getroot()
        model=root.find('model'); model.remove(next(n for n in model if n.get('name')==m.ODOM_PLUGIN))
        tree.write(p)
        self.assertFalse(m.run(self.args)['generated_sdf']['summit0']['passed'])

    def test_source_xacro_plugin_frame_change_rejected(self):
        self.source.write_text(XACRO.replace('${ns}/base_footprint','base_link'))
        self.assertFalse(m.run(self.args)['source_configuration']['passed'])

    def test_launch_routing_change_rejected(self):
        self.launch.write_text(LAUNCH.replace('"encoder_odom"','"odom"'))
        self.assertFalse(m.run(self.args)['source_configuration']['passed'])

    def test_bad_dynamic_report_rejected(self):
        doc=dynamic();doc['robots']['summit0']['max_xy_delta_m']=.5
        self.dyn.write_text(json.dumps(doc))
        self.assertFalse(m.run(self.args)['dynamic_observation']['passed'])

    def test_static_dynamic_report_rejected(self):
        doc=dynamic();doc['status']='STATIC_FRAME_CONSISTENCY_OBSERVED_NOT_PROVEN'
        self.dyn.write_text(json.dumps(doc))
        self.assertFalse(m.run(self.args)['dynamic_observation']['passed'])

    def test_late_warnings_rejected(self):
        doc=dynamic();doc['capture_warnings']=['some missing samples']
        self.dyn.write_text(json.dumps(doc))
        self.assertFalse(m.run(self.args)['dynamic_observation']['passed'])

if __name__=='__main__':unittest.main()
