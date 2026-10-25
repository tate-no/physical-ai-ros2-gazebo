from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


class AssetTests(unittest.TestCase):
    def test_visuals_use_confirmed_docker_mesh_layout(self):
        package = Path(__file__).resolve().parents[1]
        model = ET.parse(package / 'models/burger_contact.sdf').getroot().find('model')
        expected = {'base_link': 'burger_base.dae', 'base_scan': 'lds.dae',
                    'wheel_left_link': 'tire.dae',
                    'wheel_right_link': 'tire.dae'}
        for name, suffix in expected.items():
            visual = model.find(f"link[@name='{name}']/visual")
            self.assertEqual(visual.findtext('geometry/mesh/uri'),
                             'model://turtlebot3_common/meshes/' + suffix)
            self.assertIsNotNone(visual.find('material/diffuse'))

    def test_spawn_xml_has_no_encoding_declaration(self):
        package = Path(__file__).resolve().parents[1]
        for relative in ('models/burger_contact.sdf', 'worlds/color_targets.world'):
            xml = (package / relative).read_text(encoding='utf-8')
            self.assertFalse(xml.lstrip().startswith('<?xml'), relative)
            self.assertEqual(ET.fromstring(xml).tag, 'sdf')

    def test_sensor_geometry_and_contact_configuration(self):
        package = Path(__file__).resolve().parents[1]
        model = ET.parse(package / 'models/burger_contact.sdf').getroot().find('model')
        sensor = model.find("link[@name='base_link']/sensor[@type='contact']")
        self.assertEqual(sensor.findtext('contact/collision'), 'touch_bumper')
        self.assertEqual(sensor.findtext('plugin/ros/remapping'), 'bumper_states:=contacts')
        joint = [float(v) for v in model.findtext("joint[@name='lidar_joint']/pose").split()]
        ray = [float(v) for v in model.findtext(".//sensor[@type='ray']/pose").split()]
        self.assertLess(joint[2] + ray[2], .2)
        self.assertGreater(joint[2] + ray[2], .1)
        # The reduced ray minimum must not expose the enclosing LDS cylinder.
        self.assertIsNone(model.find("link[@name='base_scan']/collision"))
        self.assertIsNotNone(model.find("link[@name='base_scan']/visual"))
        self.assertIsNotNone(model.find("link[@name='base_link']/collision[@name='base_collision']"))
        self.assertEqual(len(ET.parse(package / 'worlds/color_targets.world').findall('.//model')), 4)
