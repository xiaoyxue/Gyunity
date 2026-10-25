import hashlib
import os
from pathlib import Path
import tempfile
import unittest

from gyscene import Conductor, Scene
from gyscene.pbrt import import_file, tokenize

PROJECT = Path(__file__).resolve().parents[1]
OPTIONS = '''
LookAt 0 0 -4 0 0 0 0 1 0
Camera "perspective" "float fov" 40
Film "rgb" "integer xresolution" 320 "integer yresolution" 240
Sampler "independent" "integer pixelsamples" 8
Integrator "path" "integer maxdepth" 5
PixelFilter "box"
WorldBegin
'''
LIGHT = '''
AttributeBegin
AreaLightSource "diffuse" "rgb L" [5 5 5]
Translate 0 3 0
Shape "sphere" "float radius" .3
AttributeEnd
'''


class PbrtTests(unittest.TestCase):
    def setUp(self):
        output = os.environ.get("GYUNITY_TEST_OUTPUT_DIRECTORY")
        if not output:
            self.skipTest("Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main checkout Test")
        self.temporary = tempfile.TemporaryDirectory(dir=output)
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def convert(self, text):
        source = self.root / "scene.pbrt"
        source.write_text(text, encoding="utf-8")
        return import_file(source, self.root / "out", rgb_radiance_scale=1.)

    def test_native_model_and_deterministic_original_source_retention(self):
        source = PROJECT / "examples" / "pbrt_demo.pbrt"
        output = import_file(source, self.root / "out", rgb_radiance_scale=1.)
        first = output.read_bytes()
        self.assertEqual(first, import_file(source, self.root / "out", rgb_radiance_scale=1.).read_bytes())
        self.assertEqual((output.parent / "sources" / source.name).read_bytes(), source.read_bytes())
        scene = Scene.read_json(output)
        self.assertEqual(len(scene.shapes), 5)
        self.assertEqual(scene.render.seed, 123)
        conductor = next(m for m in scene.materials.values() if isinstance(m, Conductor))
        self.assertEqual(conductor.alpha, (0.2, 0.2))
        self.assertEqual(scene.metadata["source_files"][source.name], hashlib.sha256(source.read_bytes()).hexdigest())
        self.assertEqual(scene.shapes[-1].emission, (1., 1., 1.))
        with self.assertRaises(ValueError):
            import_file(source, self.root / "missing-conversion")

    def test_column_major_transform_and_attribute_scope(self):
        output = self.convert(OPTIONS + '''
AttributeBegin
Transform [1 0 0 0 0 1 0 0 0 0 1 0 2 0 0 1]
Scale 2 2 2
Shape "sphere" "float radius" .5
AttributeEnd
Shape "sphere" "float radius" .5
''' + LIGHT)
        scene = Scene.read_json(output)
        self.assertEqual(scene.shapes[0].center, (2., 0., 0.))
        self.assertEqual(scene.shapes[0].radius, 1.)
        self.assertEqual(scene.shapes[1].center, (0., 0., 0.))
        self.assertEqual(scene.camera.to_world[:3], (1., 0., 0.))
        self.assertEqual(scene.camera.to_world[11], -4.)

    def test_includes_resolve_from_initial_file_and_retain_named_material(self):
        (self.root / "parts").mkdir()
        (self.root / "parts" / "geometry.pbrt").write_text('NamedMaterial "red"\nShape "sphere"\n', encoding="utf-8")
        scene = Scene.read_json(self.convert(OPTIONS + '''
MakeNamedMaterial "red" "string type" "diffuse" "rgb reflectance" [.7 .1 .1]
Include "parts/geometry.pbrt"
''' + LIGHT))
        self.assertEqual(scene.materials[scene.shapes[0].material].reflectance, (0.7, 0.1, 0.1))
        self.assertIn("parts/geometry.pbrt", scene.metadata["source_files"])

    def test_unsupported_and_invalid_input_never_falls_back(self):
        for text in (
            'Material "coateddiffuse"\nShape "sphere"',
            'Shape "disk"',
            'Scale 1 2 1\nShape "sphere"',
            'ReverseOrientation\nShape "sphere"',
            'Material "conductor" "spectrum eta" "metal-Cu-eta"',
            'AttributeEnd',
            'Shape "sphere" "float radius" -.5',
            'Shape "sphere" "float unknown" 2',
        ):
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.convert(OPTIONS + text + LIGHT)
        with self.assertRaises(ValueError):
            self.convert(OPTIONS.replace('Sampler "independent" "integer pixelsamples" 8', '') + LIGHT)
        with self.assertRaises(ValueError):
            self.convert(OPTIONS + 'Include "scene.pbrt"')

    def test_tokenizer_handles_comments_and_rejects_unterminated_strings(self):
        tokens = tokenize('Translate 1# comment\n 2 3', "example")
        self.assertEqual([t.text for t in tokens], ["Translate", "1", "2", "3"])
        with self.assertRaises(ValueError):
            tokenize('Shape "unterminated', "example")

    def test_ply_mesh_preserves_positions_and_topology(self):
        try:
            import plyfile
        except ImportError:
            self.skipTest("Run with uv --extra pbrt for PLY coverage")
        (self.root / "mesh.ply").write_text(
            "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\n"
            "element face 1\nproperty list uchar int vertex_indices\nend_header\n"
            "-1 0 0\n1 0 0\n0 1 0\n3 0 1 2\n", encoding="ascii")
        output = self.convert(OPTIONS + 'Shape "plymesh" "string filename" "mesh.ply"\n' + LIGHT)
        scene = Scene.read_json(output)
        mesh = output.parent.joinpath(*scene.shapes[0].file.split("\\"))
        self.assertIn("v -1 0 0\n", mesh.read_text())
        self.assertIn("f 1 2 3\n", mesh.read_text())
        self.assertEqual((output.parent / "sources" / "mesh.ply").read_bytes(), (self.root / "mesh.ply").read_bytes())
