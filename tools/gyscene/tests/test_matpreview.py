import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from pydantic import ValidationError
from gyscene import Camera, Checkerboard, Diffuse, Environment, Mirror, Plastic, RoughDielectric, Scene, Sphere
from gyscene.matpreview import PROFILES, import_presets, import_preview, import_previews, profile_material, source_data


class PreviewModelTests(unittest.TestCase):
    def test_supported_materials_and_environment_are_explicit_native_types(self):
        checker = Checkerboard(color0=(0.4, 0.4, 0.4), color1=(0.2, 0.2, 0.2), scale=(8., 8.))
        self.assertFalse(Diffuse(reflectance=checker, two_sided=False).two_sided)
        self.assertEqual(Environment(file="environment.exr").scale, 1.)
        with self.assertRaises(ValidationError):
            Environment(file=r"..\outside.exr")
        with self.assertRaises(ValidationError):
            Checkerboard(color0=(2., 0., 0.), color1=(0., 0., 0.))

    def test_all_existing_material_families_have_native_models(self):
        materials = {
            "mirror": Mirror(reflection=(0.8, 0.7, 0.6)),
            "rough-glass": RoughDielectric(roughness=(0.1, 0.3), eta_inside=1.9),
            "plastic": Plastic(diffuse=(0.2, 0.7, 0.3), specular=(0.5, 0.5, 0.5), roughness=0.2),
        }
        scene = Scene(name="materials", camera=Camera.look_at((0., 0., 3.), (0., 0., 0.)), materials=materials)
        scene.add(Sphere(id="emitter", material="mirror", emission=(1., 1., 1.)))
        self.assertEqual(Scene.model_validate_json(scene.to_json()).materials, materials)
        for value in (0., -0.1, 1.1, float("inf")):
            with self.assertRaises(ValidationError):
                RoughDielectric(roughness=(value, 0.1))
            with self.assertRaises(ValidationError):
                Plastic(diffuse=(0.2, 0.2, 0.2), roughness=value)
        for material in (
            {"type": "mirror", "reflection": [2., 1., 1.]},
            {"type": "rough_dielectric", "roughness": [0.1, 0.1], "eta_outside": 1.3},
            {"type": "rough_dielectric", "roughness": [0.1, 0.1], "eta_inside": 0},
            {"type": "plastic", "diffuse": [0.2, 0.2, 0.2], "roughness": 0.1, "distribution": "beckmann"},
        ):
            data = scene.model_dump(mode="json")
            data["materials"] = {"mirror": material}
            with self.assertRaises(ValidationError):
                Scene.model_validate_json(json.dumps(data))

    def test_bad_profiles_fail_before_loading_assets(self):
        for profiles in ({}, {"../escape": Mirror()}, {"invalid": object()}):
            with self.assertRaises(ValueError):
                import_previews(Path("missing.zip"), Path("unused"), profiles=profiles, max_depth=8)
        with self.assertRaises(ValueError):
            import_preview(Path("missing.zip"), Path("unused"), profile_name="mirror", exterior=Mirror(), max_depth=0)


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
            import mitsuba as mi
            with patch.object(mi, "load_file", wraps=mi.load_file) as decode:
                paths = import_presets(archive_file, Path(directory), profile="all", max_depth=8)
                self.assertEqual(decode.call_count, 1)
            self.assertEqual(len(paths), 9)
            self.assertEqual(len(set(PROFILES)), 9)
            types = set()
            for profile, path in zip(PROFILES, paths):
                scene = Scene.read_json(path)
                exterior = next(iter(scene.metadata["material_overrides"].values()))["replacement"]
                types.add(exterior["type"])
                self.assertEqual(exterior, profile_material(profile, root).model_dump(mode="json"))
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
            self.assertEqual(types, {"diffuse", "dielectric", "conductor", "mirror", "rough_dielectric", "plastic"})
            self.assertEqual(profile_material("copper", root).alpha, (0.1, 0.1))
            self.assertEqual(profile_material("copper", root).distribution, "ggx")
            self.assertEqual(profile_material("copper-beckmann", root).distribution, "beckmann")
            self.assertNotEqual(*profile_material("copper-anisotropic", root).alpha)
            self.assertEqual(profile_material("glass", root).eta_inside, 1.9)
            self.assertEqual(profile_material("diffuse", root).reflectance, (0.240, 0.771, 0.361))
            batch_diffuse = paths[0].read_bytes()
            path = import_preview(archive_file, Path(directory), profile_name="diffuse",
                                  exterior=profile_material("diffuse", root), max_depth=8)
            first = path.read_bytes()
            self.assertEqual(first, batch_diffuse)
            import_preview(archive_file, Path(directory), profile_name="diffuse",
                           exterior=profile_material("diffuse", root), max_depth=8)
            self.assertEqual(first, path.read_bytes())
