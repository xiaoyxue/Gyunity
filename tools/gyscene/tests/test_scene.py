import json
import os
from pathlib import Path
import runpy
import tempfile
import unittest
from zipfile import ZipFile

from pydantic import ValidationError
from gyscene import Camera, Conductor, Diffuse, Mesh, Quad, Scene, Sphere
from gyscene.mitsuba import ARCHIVES, import_archive, import_xml, matrix, parse_scene
import xml.etree.ElementTree as ET

PROJECT = Path(__file__).resolve().parents[1]


class NativeSceneTests(unittest.TestCase):
    def demo(self):
        return runpy.run_path(str(PROJECT / "examples" / "native_demo.py"))["scene"]

    def test_native_authoring_is_deterministic_and_round_trips(self):
        scene = self.demo()
        text = scene.to_json()
        self.assertEqual(text, scene.to_json())
        self.assertEqual(text, Scene.model_validate_json(text).to_json())
        self.assertEqual(len(scene.shapes), 3)

    def test_invalid_contract_values_are_rejected(self):
        data = json.loads(self.demo().to_json())
        for key, value in (("schema_version", 2), ("unknown", 0)):
            changed = dict(data, **{key: value})
            with self.assertRaises(ValidationError):
                Scene.model_validate_json(json.dumps(changed))
        data["shapes"][0]["material"] = "missing"
        with self.assertRaises(ValidationError):
            Scene.model_validate_json(json.dumps(data))
        with self.assertRaises(ValidationError):
            Diffuse(reflectance=(1.1, 0., 0.))
        with self.assertRaises(ValidationError):
            Conductor(distribution="beckmann", alpha=(0., 0.1), eta=(1., 1., 1.), k=(1., 1., 1.))
        with self.assertRaises(ValueError):
            Camera.look_at((0., 0., 0.), (0., 0., 0.))

    def test_mutated_references_and_duplicate_ids_are_checked_before_output(self):
        scene = self.demo()
        scene.shapes.append(scene.shapes[0])
        with self.assertRaises(ValueError):
            scene.to_json()

    def test_look_at_uses_native_right_up_forward_columns(self):
        camera = Camera.look_at((0., 0., 3.), (0., 0., 0.))
        self.assertEqual(camera.to_world, (1., 0., 0., 0., 0., 1., 0., 0., 0., 0., -1., 3., 0., 0., 0., 1.))

    def test_invalid_paths_geometry_and_numbers_fail(self):
        for path in (r"..\secret.obj", r"C:\private.obj", r"\root.obj"):
            with self.assertRaises(ValidationError):
                Mesh(id="mesh", material="white", file=path)
        with self.assertRaises(ValidationError):
            Quad(id="quad", material="white", vertices=((0., 0., 0.),) * 4)
        with self.assertRaises(ValidationError):
            Sphere(id="sphere", material="white", radius=float("inf"))
        data = json.loads(self.demo().to_json())
        data["camera"]["to_world"] = [0.] * 16
        with self.assertRaises(ValidationError):
            Scene.model_validate_json(json.dumps(data))

    def test_schema_exposes_native_types_and_forbids_unknown_fields(self):
        schema = Scene.model_json_schema()
        self.assertFalse(schema["additionalProperties"])
        self.assertEqual(schema["properties"]["schema_version"]["const"], 1)
        self.assertIn("Box", schema["$defs"])
        self.assertIn("Conductor", schema["$defs"])

    def test_mitsuba_transform_operations_follow_source_order(self):
        result = matrix(ET.fromstring('<transform name="to_world"><translate x="2"/><scale value="3"/></transform>'))
        self.assertEqual(result[3], 6.)
        camera = matrix(ET.fromstring('<transform name="to_world"><lookat origin="0,0,3" target="0,0,0" up="0,1,0"/></transform>'))
        self.assertEqual(camera[0], -1.)
        self.assertEqual(camera[10], -1.)
        self.assertEqual(camera[11], 3.)


@unittest.skipUnless(os.environ.get("GYUNITY_MITSUBA_ARCHIVES"), "Set GYUNITY_MITSUBA_ARCHIVES for original-data checks")
class GalleryTests(unittest.TestCase):
    def test_all_original_archives_import_without_transcribed_scene_values(self):
        output = os.environ.get("GYUNITY_TEST_OUTPUT_DIRECTORY")
        self.assertTrue(output, "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main checkout Test")
        with tempfile.TemporaryDirectory(dir=output) as temp:
            for key, (name, _) in ARCHIVES.items():
                archive_path = Path(os.environ["GYUNITY_MITSUBA_ARCHIVES"]) / (key + ".zip")
                result = import_archive(archive_path, Path(temp))
                scene = Scene.read_json(result)
                first = result.read_bytes()
                import_archive(archive_path, Path(temp))
                self.assertEqual(first, result.read_bytes())
                self.assertEqual(scene.name, name)
                self.assertEqual(scene.render.filter, "tent")
                with ZipFile(archive_path) as archive:
                    self.assertEqual((result.parent / "scene.xml").read_bytes(), archive.read(key + "/scene.xml"))
                    self.assertEqual((result.parent / "LICENSE.txt").read_bytes(), archive.read(key + "/LICENSE.txt"))
                    for shape in scene.shapes:
                        if shape.type == "mesh":
                            member = key + "/" + shape.file.replace("\\", "/")
                            self.assertEqual(result.parent.joinpath(*shape.file.split("\\")).read_bytes(), archive.read(member))
                    xml = archive.read(key + "/scene.xml")
                    with self.assertRaises(ValueError):
                        parse_scene(xml.replace(b'<scene version="3.0.0">', b'<scene version="3.0.0"><unknown/>'), name)
                direct = import_xml(result.parent / "scene.xml", Path(temp) / "direct", name)
                self.assertEqual(Scene.read_json(direct).shapes, scene.shapes)
                self.assertEqual(Scene.read_json(direct).camera, scene.camera)
                self.assertEqual(len(scene.shapes), {"cornell-box": 8, "veach-mis": 9, "veach-bidir": 14}[key])
                if key == "veach-mis":
                    self.assertEqual(sum(s.type == "sphere" for s in scene.shapes), 3)
                    self.assertEqual(sorted(m.alpha[0] for m in scene.materials.values() if isinstance(m, Conductor)), [0.01, 0.05, 0.1, 0.25])


if __name__ == "__main__":
    unittest.main()
