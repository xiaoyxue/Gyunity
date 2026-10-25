import importlib.util
import os
from pathlib import Path
import tempfile
import unittest

from pydantic import ValidationError
from gyscene import Checkerboard, Diffuse, Environment, Scene
from gyscene.matpreview import PROFILES, import_preview, profile_material, source_data


class PreviewModelTests(unittest.TestCase):
    def test_supported_materials_and_environment_are_explicit_native_types(self):
        checker = Checkerboard(color0=(0.4, 0.4, 0.4), color1=(0.2, 0.2, 0.2), scale=(8., 8.))
        self.assertFalse(Diffuse(reflectance=checker, two_sided=False).two_sided)
        self.assertEqual(Environment(file="environment.exr").scale, 1.)
        with self.assertRaises(ValidationError):
            Environment(file=r"..\outside.exr")
        with self.assertRaises(ValidationError):
            Checkerboard(color0=(2., 0., 0.), color1=(0., 0., 0.))


@unittest.skipUnless(os.environ.get("GYUNITY_MATPREVIEW_ARCHIVE") and importlib.util.find_spec("mitsuba"),
                     "Set GYUNITY_MATPREVIEW_ARCHIVE and install the mitsuba uv extra for original-data checks")
class MatpreviewTests(unittest.TestCase):
    def test_original_geometry_and_exr_are_retained_for_supported_profiles(self):
        output = os.environ.get("GYUNITY_TEST_OUTPUT_DIRECTORY")
        self.assertTrue(output, "Set the main checkout Test output directory")
        archive_file = Path(os.environ["GYUNITY_MATPREVIEW_ARCHIVE"])
        root, original = source_data(archive_file)
        with tempfile.TemporaryDirectory(dir=output) as directory:
            previous_shapes = None
            for profile in PROFILES:
                path = import_preview(archive_file, Path(directory), profile_name=profile,
                                      exterior=profile_material(profile, root), max_depth=8)
                scene = Scene.read_json(path)
                self.assertEqual(scene.name, "matpreview-" + profile)
                self.assertEqual((scene.render.width, scene.render.height), (683, 512))
                self.assertEqual(scene.render.filter, "gaussian")
                self.assertEqual(scene.metadata["source_max_depth"], -1)
                self.assertEqual(len(scene.shapes), 3)
                self.assertEqual(len(scene.lights), 1)
                self.assertEqual(scene.lights[0].scale, 3.)
                self.assertEqual(sum(v["triangles"] for v in scene.metadata["world_space_meshes"].values()), 61600)
                if previous_shapes is not None:
                    self.assertEqual(scene.shapes, previous_shapes)
                previous_shapes = scene.shapes
                for name, data in original.items():
                    self.assertEqual((path.parent / "source" / name).read_bytes(), data)
                self.assertIn("material_overrides", scene.metadata)
                self.assertNotIn("CC0", scene.metadata["license"])
            path = import_preview(archive_file, Path(directory), profile_name="diffuse",
                                  exterior=profile_material("diffuse", root), max_depth=8)
            first = path.read_bytes()
            import_preview(archive_file, Path(directory), profile_name="diffuse",
                           exterior=profile_material("diffuse", root), max_depth=8)
            self.assertEqual(first, path.read_bytes())
