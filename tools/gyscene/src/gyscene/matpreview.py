"""Local import of the original matpreview asset with explicit native material profiles."""

import hashlib
import math
from pathlib import Path, PurePosixPath
import re
from zipfile import ZipFile
import xml.etree.ElementTree as ET

from pydantic import TypeAdapter

from .meshio import encode_obj
from .model import Camera, Checkerboard, Conductor, Dielectric, Diffuse, Environment, Material, Mesh, Mirror, Plastic, RenderSettings, RoughDielectric, Scene
from .transforms import multiply

ARCHIVE_SHA256 = "0dd0b3303e7b3f97995bb37490ae05256c93d6a2e618eeef9adeee54bd838587"
PROFILES = ("diffuse", "glass", "copper", "checkerboard", "mirror", "rough-glass",
            "copper-beckmann", "copper-anisotropic", "plastic")


def rgb(node):
    values = tuple(float(x) for x in node.attrib["value"].replace(",", " ").split())
    if len(values) == 1:
        values *= 3
    if len(values) != 3:
        raise ValueError("Expected an RGB source value")
    return values


def source_data(archive_file: Path):
    if hashlib.sha256(archive_file.read_bytes()).hexdigest() != ARCHIVE_SHA256:
        raise ValueError("matpreview.zip does not match the reviewed source archive")
    with ZipFile(archive_file) as archive:
        xml = archive.read("matpreview/scene.xml")
        root = ET.fromstring(xml)
        names = {"scene.xml"}
        names.update(node.attrib["value"] for node in root.findall(".//string[@name='filename']"))
        payload = {}
        for name in sorted(names):
            path = PurePosixPath(name)
            if path.is_absolute() or ".." in path.parts or ":" in name or "\\" in name:
                raise ValueError(f"Unsafe source asset path: {name}")
            payload[name] = archive.read("matpreview/" + name)
    return root, payload


def profile_material(profile: str, root):
    plastics = [node for node in root.findall("bsdf") if node.get("type") == "plastic"]
    if len(plastics) != 1:
        raise ValueError("The reviewed material-preview source must have one plastic exterior")
    source = plastics[0]
    if profile == "diffuse":
        return Diffuse(reflectance=rgb(source.find("rgb[@name='diffuse_reflectance']")), two_sided=False)
    if profile == "glass":
        return Dielectric(eta_inside=float(source.find("float[@name='int_ior']").attrib["value"]))
    if profile in ("copper", "copper-beckmann", "copper-anisotropic"):
        # An explicit native demonstration preset, not an approximation of source plastic.
        return Conductor(distribution="beckmann" if profile == "copper-beckmann" else "ggx",
                         alpha=(0.05, 0.25) if profile == "copper-anisotropic" else (0.1, 0.1),
                         eta=(0.200438, 0.924033, 1.10221), k=(3.91295, 2.45285, 2.14219))
    if profile == "checkerboard":
        return Diffuse(reflectance=Checkerboard(color0=(0.8, 0.8, 0.8), color1=(0.05, 0.05, 0.05),
                                               scale=(8., 8.)), two_sided=False)
    if profile == "mirror":
        return Mirror()
    if profile == "rough-glass":
        return RoughDielectric(roughness=(0.1, 0.1),
                               eta_inside=float(source.find("float[@name='int_ior']").attrib["value"]))
    if profile == "plastic":
        return Plastic(diffuse=rgb(source.find("rgb[@name='diffuse_reflectance']")),
                       eta_inside=float(source.find("float[@name='int_ior']").attrib["value"]),
                       eta_outside=1.000277, nonlinear=False)
    raise ValueError(f"Unsupported material profile: {profile}")


def preserve(path: Path, data: bytes):
    if path.exists() and path.read_bytes() != data:
        raise FileExistsError(f"Refusing to overwrite different source/generated data: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def import_preview(archive_file: Path, output_root: Path, *, profile_name: str,
                   exterior: Material, max_depth: int) -> Path:
    """Import one explicitly supplied material, using the shared batch decoder."""
    return import_previews(archive_file, output_root, profiles={profile_name: exterior}, max_depth=max_depth)[0]


def import_presets(archive_file: Path, output_root: Path, *, profile: str, max_depth: int) -> list[Path]:
    names = PROFILES if profile == "all" else (profile,)
    root, _ = source_data(archive_file)
    profiles = {name: profile_material(name, root) for name in names}
    return import_previews(archive_file, output_root, profiles=profiles, max_depth=max_depth)


def import_previews(archive_file: Path, output_root: Path, *, profiles: dict[str, Material],
                    max_depth: int) -> list[Path]:
    """Decode shared original assets once; replace only each explicitly supplied exterior BSDF."""
    if not profiles:
        raise ValueError("Choose at least one explicit material profile")
    for name in profiles:
        if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
            raise ValueError("Profile name must contain only letters, digits, underscores or hyphens")
    adapter = TypeAdapter(Material)
    profiles = {name: adapter.validate_python(material) for name, material in profiles.items()}
    if not isinstance(max_depth, int) or isinstance(max_depth, bool) or max_depth <= 0:
        raise ValueError("Choose an explicit positive depth budget for the unlimited source")
    try:
        import mitsuba as mi
        import numpy as np
    except ImportError as error:
        raise ValueError("Material-preview decoding requires uv run --extra mitsuba ...") from error
    root, payload = source_data(archive_file)
    destination = output_root / "matpreview"
    for name, data in payload.items():
        preserve(destination / "source" / name, data)

    mi.set_variant("scalar_rgb")
    decoded = mi.load_file(str(destination / "source" / "scene.xml"))
    sensor = decoded.sensors()[0]
    camera_matrix = np.asarray(sensor.world_transform().matrix, dtype=np.float32).reshape(4, 4).astype(float)
    camera_matrix[:3, 0] *= -1
    width, height = (int(x) for x in sensor.film().size())
    sensor_xml = root.find("sensor")
    fov = float(sensor_xml.find("float[@name='fov']").attrib["value"])
    if sensor_xml.find("string[@name='fov_axis']").attrib["value"] != "smaller":
        raise ValueError("Unexpected source camera convention")
    fov_y = fov if width >= height else math.degrees(2 * math.atan(math.tan(math.radians(fov)/2) * height/width))
    defaults = {node.attrib["name"]: node.attrib["value"] for node in root.findall("default")}
    scene = Scene(name="matpreview",
                  camera=Camera(to_world=tuple(float(x) for x in camera_matrix.ravel()), fov_y=fov_y),
                  render=RenderSettings(width=width, height=height, spp=int(defaults["spp"]),
                                        max_depth=max_depth, filter="gaussian"))
    textures = {}
    for node in root.findall("texture"):
        if node.get("type") != "checkerboard":
            raise ValueError("Unsupported material-preview texture")
        scale = node.find("transform/scale")
        textures[node.attrib["id"]] = Checkerboard(
            color0=rgb(node.find("rgb[@name='color0']")), color1=rgb(node.find("rgb[@name='color1']")),
            scale=(float(scale.attrib["x"]), float(scale.attrib["y"])))
    overrides = {}
    for node in root.findall("bsdf"):
        identifier = node.attrib["id"]
        if node.get("type") == "diffuse":
            reference = node.find("ref[@name='reflectance']")
            color = textures[reference.attrib["id"]] if reference is not None else rgb(node.find("rgb[@name='reflectance']"))
            material = Diffuse(reflectance=color, two_sided=False)
        elif node.get("type") == "plastic":
            material = next(iter(profiles.values()))
            overrides[identifier] = {"source_type": "plastic", "replacement": material.model_dump(mode="json")}
        else:
            raise ValueError(f"Unsupported source material: {node.get('type')}")
        scene.add_material(identifier, material)

    meshes = {shape.id(): shape for shape in decoded.shapes()}
    source_meshes = {}
    for node in root.findall("shape"):
        identifier = node.attrib["id"]
        if node.get("type") != "serialized":
            raise ValueError("Unexpected material-preview shape type")
        shape = meshes[identifier]
        parameters = mi.traverse(shape)
        positions = np.asarray(parameters["vertex_positions"], dtype=np.float32).reshape(-1, 3)
        normals = np.asarray(parameters["vertex_normals"], dtype=np.float32).reshape(-1, 3)
        uv = np.asarray(parameters["vertex_texcoords"], dtype=np.float32).reshape(-1, 2)
        faces = np.asarray(parameters["faces"], dtype=np.uint32).reshape(-1, 3)
        relative = "meshes\\" + identifier + ".obj"
        data = encode_obj(tuple(float(x) for x in positions.ravel()), tuple(int(x) for x in faces.ravel()),
                          tuple(float(x) for x in normals.ravel()), tuple(float(x) for x in uv.ravel()))
        preserve(destination / "meshes" / (identifier + ".obj"), data)
        # Mitsuba's mesh buffers already contain source to_world transforms.
        scene.add(Mesh(id=identifier, file=relative, material=node.find("ref[@name='bsdf']").attrib["id"]))
        source_meshes[identifier] = {"vertices": len(positions), "triangles": len(faces),
                                     "bounds_min": positions.min(axis=0).astype(float).tolist(),
                                     "bounds_max": positions.max(axis=0).astype(float).tolist(),
                                     "obj_sha256": hashlib.sha256(data).hexdigest()}

    emitter = decoded.environment()
    env_xml = root.find("emitter")
    if env_xml is None or env_xml.get("type") != "envmap":
        raise ValueError("Expected the original EXR environment")
    original_matrix = np.asarray(emitter.world_transform().matrix, dtype=np.float32).reshape(4, 4).astype(float)
    direction_matrix = original_matrix.copy()
    direction_matrix[:3, 3] = 0
    # Native u=0 points +X; Mitsuba u=0 points -Z. No image resampling is needed.
    native_to_mitsuba = (0., 0., 1., 0., 0., 1., 0., 0., -1., 0., 0., 0., 0., 0., 0., 1.)
    direction_matrix = multiply(tuple(float(x) for x in direction_matrix.ravel()), native_to_mitsuba)
    scene.add_light(Environment(id=env_xml.attrib["id"],
        file="source\\" + env_xml.find("string[@name='filename']").attrib["value"],
        scale=float(env_xml.find("float[@name='scale']").attrib["value"]), to_world=direction_matrix))
    environment_samples = []
    for direction in ((1.,0.,0.), (-1.,0.,0.), (0.,1.,0.), (0.,-1.,0.), (0.,0.,1.), (0.,0.,-1.)):
        interaction = mi.SurfaceInteraction3f()
        interaction.wi = -mi.Vector3f(direction)
        environment_samples.append({"direction": list(direction), "radiance": [float(x) for x in emitter.eval(interaction)]})
    scene.metadata = {
        "source_format": "mitsuba-3", "source_archive": archive_file.name,
        "archive_sha256": ARCHIVE_SHA256, "mitsuba_decoder_version": mi.__version__,
        "license": "No license included in source archive; local import only, no redistribution asserted.",
        "source_max_depth": int(defaults["max_depth"]), "native_depth_override": max_depth,
        "material_overrides": overrides, "world_space_meshes": source_meshes,
        "source_environment_to_world": original_matrix.ravel().tolist(),
        "expected_environment_samples": environment_samples,
        "source_files": {name: hashlib.sha256(data).hexdigest() for name, data in payload.items()},
        "note": "Original geometry, camera, checkerboard and EXR are retained. Exterior material is an explicit supported profile, not source plastic."
    }
    outputs = []
    for profile_name, exterior in profiles.items():
        variant = scene.model_copy(deep=True)
        variant.name = "matpreview-" + profile_name
        for identifier in overrides:
            variant.materials[identifier] = exterior
            variant.metadata["material_overrides"][identifier]["replacement"] = exterior.model_dump(mode="json")
        if isinstance(exterior, Plastic):
            variant.metadata["note"] = (
                "Original geometry, camera, checkerboard and EXR are retained. "
                "Exterior uses native smooth layered Plastic; explicit coating parameters are recorded.")
        output = destination / (variant.name + ".json")
        variant.write_json(output)
        outputs.append(output)
    return outputs
