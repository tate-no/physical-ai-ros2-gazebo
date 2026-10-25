"""Reproduce derived assets without modifying the original model."""
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
package = root / 'src/color_target_nav'
tree = ET.parse(root / 'camera_models/turtlebot3_burger_camera.sdf')
model = tree.getroot().find('model')
# Use the DAE layout confirmed in the user's existing Docker workspace.
# Its turtlebot3_gazebo assets differ from the current upstream Humble tree.
mesh_paths = {
    'base_link': ('burger_base.dae', '0.3 0.3 0.3 1'),
    'base_scan': ('lds.dae', '0.2 0.2 0.2 1'),
    'wheel_left_link': ('tire.dae', '0.2 0.2 0.2 1'),
    'wheel_right_link': ('tire.dae', '0.2 0.2 0.2 1'),
}
for link_name, (mesh_path, color) in mesh_paths.items():
    visual = model.find(f"link[@name='{link_name}']/visual")
    visual.find('geometry/mesh/uri').text = 'model://turtlebot3_common/meshes/' + mesh_path
    material = ET.SubElement(visual, 'material')
    ET.SubElement(material, 'ambient').text = color
    ET.SubElement(material, 'diffuse').text = color
base = model.find("link[@name='base_link']")
base.append(ET.fromstring('''<collision name="touch_bumper">
  <pose>0.14 0 0.08 0 0 0</pose>
  <geometry><box><size>0.10 0.14 0.06</size></box></geometry>
</collision>'''))
base.append(ET.fromstring('''<visual name="touch_bumper_visual">
  <pose>0.14 0 0.08 0 0 0</pose>
  <geometry><box><size>0.10 0.14 0.06</size></box></geometry>
  <material><ambient>0.3 0.3 0.3 1</ambient></material>
</visual>'''))
base.append(ET.fromstring('''<sensor name="touch" type="contact">
  <always_on>true</always_on><update_rate>30</update_rate>
  <contact><collision>touch_bumper</collision></contact>
  <plugin name="contact_ros" filename="libgazebo_ros_bumper.so">
    <ros><namespace>/color_nav</namespace><remapping>bumper_states:=contacts</remapping></ros>
    <frame_name>base_link</frame_name>
  </plugin>
</sensor>'''))
# Original model places the LiDAR height both on joint and sensor. Move it
# into the link transform once so its rays intersect the 0.2 m boxes.
joint = model.find("joint[@name='lidar_joint']/pose")
joint.text = '0 0 0 0 0 0'
model.find(".//sensor[@type='ray']/ray/range/min").text = '0.03'
# With a 3 cm minimum the ray origin lies inside the original LDS cylinder
# (radius 5.08 cm), so rays report that cylinder instead of the world.
# Keep its visual/inertia, but omit its collision in this derived demo model.
# The body, wheels and contact bumper retain their physical collisions.
scan_link = model.find("link[@name='base_scan']")
scan_link.remove(scan_link.find("collision[@name='lidar_sensor_collision']"))
model.find(".//sensor[@type='camera']/camera/clip/near").text = '0.01'
# Humble spawn_entity reads a Unicode string and passes it to lxml. An
# encoding declaration on that string is rejected before the spawn service.
tree.write(package / 'models/burger_contact.sdf', encoding='unicode', xml_declaration=False)
world = ET.Element('sdf', version='1.6')
w = ET.SubElement(world, 'world', name='color_targets')
for uri in ('model://sun', 'model://ground_plane'):
    ET.SubElement(ET.SubElement(w, 'include'), 'uri').text = uri
ET.SubElement(w, 'physics', name='default_physics', type='ode')
for color in ('red', 'blue', 'yellow'):
    w.append(ET.parse(root / f'camera_models/{color}_box.sdf').getroot().find('model'))
# Obstacle off the direct initial red approach; movable layouts can be tested manually.
w.append(ET.fromstring('''<model name="gray_obstacle"><static>true</static>
  <pose>1.5 0.8 0.15 0 0 0</pose><link name="link">
  <collision name="collision"><geometry><box><size>0.3 0.3 0.3</size></box></geometry></collision>
  <visual name="visual"><geometry><box><size>0.3 0.3 0.3</size></box></geometry>
  <material><ambient>0.4 0.4 0.4 1</ambient></material></visual></link></model>'''))
ET.ElementTree(world).write(package / 'worlds/color_targets.world', encoding='unicode', xml_declaration=False)

# Keep regenerated XML free of inherited trailing whitespace.
for asset in (package / 'models/burger_contact.sdf', package / 'worlds/color_targets.world'):
    asset.write_text('\n'.join(line.rstrip() for line in asset.read_text(encoding='utf-8').splitlines()) + '\n',
                     encoding='utf-8')
