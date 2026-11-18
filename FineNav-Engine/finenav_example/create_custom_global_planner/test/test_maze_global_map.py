#!/usr/bin/python3
import importlib.util
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve()
spec = importlib.util.spec_from_file_location('maze_global_map', HERE.parents[1] / 'scripts/maze_global_map.py')
mapper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mapper)
RESOURCE = HERE.parents[3] / 'finenav_simulator/resource'


class MapGeometryTest(unittest.TestCase):
    def shapes(self, xml):
        return list(mapper.collision_shapes(ET.fromstring(xml), RESOURCE))

    def at(self, result, x, y):
        data, ox, oy = result
        return data[int((y-oy)/0.1), int((x-ox)/0.1)]

    def raster(self, obstacle):
        xml = '<sdf><world><model><link><collision><pose>0 0 -0.05 0 0 0</pose><geometry><box><size>4 4 0.1</size></box></geometry></collision></link></model>' + obstacle + '</world></sdf>'
        return mapper.rasterize(self.shapes(xml), 0.1)

    def test_circle_projection_preserves_free_corners(self):
        for geometry in ('<cylinder><radius>0.5</radius><length>1</length></cylinder>',
                         '<sphere><radius>0.5</radius></sphere>'):
            with self.subTest(geometry=geometry):
                result = self.raster('<model><pose>0 0 0.5 0 0 0</pose><link><collision><geometry>' + geometry + '</geometry></collision></link></model>')
                self.assertEqual(self.at(result, 0.55, 0.05), 100)
                self.assertEqual(self.at(result, 0.45, 0.45), 0)

    def test_rotated_box(self):
        result = self.raster('<model><pose>0 0 0.5 0 0 0.7853981633974483</pose><link><collision><geometry><box><size>1 0.2 1</size></box></geometry></collision></link></model>')
        self.assertEqual(self.at(result, 0.35, 0.35), 100)
        self.assertEqual(self.at(result, -0.35, 0.35), 0)

    def test_sphere_with_rolled_orientation_and_nested_offset(self):
        shapes = self.shapes('<model><pose>1 2 0.5 0 0 1.5707963267948966</pose><link><pose>1 0 0 0 0 0</pose><collision><pose>0 0 0 0.4 0.523236 0.8</pose><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model>')
        np.testing.assert_allclose(shapes[0][1][:3, 3], [1, 3, 0.5])
        np.testing.assert_allclose(mapper.vertical_extent(shapes[0]), [0, 1])

    def test_floor_and_high_obstacles_are_excluded(self):
        result = self.raster('<model><pose>0 0 3 0 0 0</pose><link><collision><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model>')
        self.assertEqual(np.count_nonzero(result[0]), 0)

    def test_unsupported_geometry_fails_explicitly(self):
        with self.assertRaisesRegex(ValueError, 'unsupported collision geometry'):
            self.shapes('<collision><geometry><mesh><uri>unknown</uri></mesh></geometry></collision>')

    def test_saved_layout_is_fixed_and_all_centers_are_mapped(self):
        world = ET.parse(RESOURCE / 'worlds/maze_with_obstacles.sdf').getroot().find('world')
        obstacles = [m for m in world.findall('model') if m.get('name') != 'flat_small_scale_maze_ground']
        self.assertEqual(len(obstacles), 43)
        self.assertNotIn('fine_robot', [m.get('name') for m in obstacles])
        result = mapper.build_map(RESOURCE, world_name='maze_with_obstacles.sdf')
        for model in obstacles:
            with self.subTest(model=model.get('name')):
                self.assertEqual(model.findtext('static'), 'true')
                x, y, *_ = map(float, model.findtext('pose').split())
                self.assertEqual(self.at(result, x, y), 100)
        self.assertEqual(self.at(result, 2, 2), 0)


if __name__ == '__main__':
    unittest.main()
