#!/usr/bin/python3
import sys
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import xml.dom.minidom as DOM

SIM = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM / 'scripts'))
from sdf_launch_utils import (generate_robot_sdf_from_xmacro, generate_robot_urdf_from_sdf,
                              normalize_robot_urdf_for_rviz)
from thirdparty.sdformat_tools import sdf_util
from thirdparty.sdformat_tools.sdf2urdf import convert_geometry


class RobotMeshConversionTest(unittest.TestCase):
    def convert(self, uri, scale=None):
        source = DOM.Document()
        geometry = source.createElement('geometry')
        mesh = source.createElement('mesh')
        geometry.appendChild(mesh)
        uri_node = source.createElement('uri')
        uri_node.appendChild(source.createTextNode(uri))
        mesh.appendChild(uri_node)
        if scale is not None:
            scale_node = source.createElement('scale')
            scale_node.appendChild(source.createTextNode(scale))
            mesh.appendChild(scale_node)
        output = DOM.Document()
        parent = output.createElement('visual')
        output.appendChild(parent)
        self.assertTrue(convert_geometry(geometry, output, parent))
        return ET.fromstring(output.toxml()).find('geometry/mesh')

    def test_file_uri_and_scale_are_preserved(self):
        mesh = self.convert('file:///tmp/support.obj', '0.095 0.07 0.095')
        self.assertEqual(mesh.get('filename'), 'file:///tmp/support.obj')
        self.assertEqual(mesh.get('scale'), '0.095 0.07 0.095')

    def test_package_uri_and_absolute_path(self):
        self.assertEqual(self.convert('package://robot/meshes/support.obj').get('filename'),
                         'package://robot/meshes/support.obj')
        mesh = self.convert('/tmp/support.obj')
        self.assertEqual(mesh.get('filename'), 'file:///tmp/support.obj')
        self.assertIsNone(mesh.get('scale'))

    def test_model_uri_resolves_existing_resource(self):
        with tempfile.TemporaryDirectory() as directory:
            model = Path(directory) / 'test_model'
            model.mkdir()
            old_paths = sdf_util.sdf_paths
            sdf_util.sdf_paths = [directory]
            try:
                self.assertEqual(self.convert('model://test_model/support.obj').get('filename'),
                                 (model / 'support.obj').as_uri())
            finally:
                sdf_util.sdf_paths = old_paths

    def test_missing_resources_fail_instead_of_generating_empty_file_uri(self):
        for uri in ('file://', 'package://', 'model://__nonexistent_robot_mesh__/support.obj'):
            with self.subTest(uri=uri), self.assertRaises(ValueError):
                self.convert(uri)

    def test_complete_robot_generation_uses_existing_mesh_at_sdf_scale(self):
        path = SIM / 'resource/robot/meshes/lidar_support_tri_prism.obj'
        sdf = generate_robot_sdf_from_xmacro(str(SIM / 'resource/robot/model.sdf.xmacro'),
                                            {'lidar_support_mesh_uri': path.as_uri()})
        urdf = normalize_robot_urdf_for_rviz(generate_robot_urdf_from_sdf(sdf))
        sdf_mesh = ET.fromstring(sdf).find('.//mesh')
        urdf_meshes = ET.fromstring(urdf).findall('.//mesh')
        self.assertEqual(len(urdf_meshes), 1)
        mesh = urdf_meshes[0]
        self.assertEqual(mesh.get('filename'), path.as_uri())
        self.assertTrue(path.is_file())
        self.assertEqual(tuple(map(float, mesh.get('scale').split())),
                         tuple(map(float, sdf_mesh.findtext('scale').split())))
        self.assertNotIn('filename="file://"', urdf)


if __name__ == '__main__':
    unittest.main()
