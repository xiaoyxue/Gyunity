"""Strict adapter for the three pinned, CC0 Mitsuba gallery packages."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path, PurePosixPath
from zipfile import ZipFile
import xml.etree.ElementTree as ET

from .model import Box, Camera, Conductor, Dielectric, Diffuse, IDENTITY, Mesh, Quad, RenderSettings, Scene, Sphere
from .transforms import look_at, multiply, rotation

ARCHIVES = {
    "cornell-box": ("CornellBoxStandard", "273bbbe0f2c319359b21488d83d3de7a098a9f7cb3270cb603347ee31a63aa08"),
    "veach-mis": ("veach-mis", "ce064bd3e7d46b318b44f02639b63ea3e1c7bdded1c5cd680e5733f6f525f3ce"),
    "veach-bidir": ("veach-bidir", "53ad0f938de0f6e6f5b980fd3ce6d64b2743e409612d2e327cd3bb63c284b321"),
}


def attributes(node: ET.Element, allowed: set[str]) -> None:
    unknown = set(node.attrib) - allowed
    if unknown:
        raise ValueError(f"<{node.tag}>: unsupported attributes {sorted(unknown)}")


def parameters(node: ET.Element, allowed: dict[str, str]) -> dict[str, ET.Element]:
    result = {}
    for child in node:
        name = child.get("name", child.tag)
        if allowed.get(name) != child.tag or name in result:
            raise ValueError(f"<{node.tag}>: unsupported or duplicate parameter {child.tag}/{name}")
        result[name] = child
    return result


def matrix(node: ET.Element | None) -> tuple[float, ...]:
    if node is None:
        return IDENTITY
    attributes(node, {"name"})
    result = None
    for operation in node:
        if len(operation):
            raise ValueError("Transform operations cannot have children")
        if operation.tag == "matrix":
            attributes(operation, {"value"})
            component = tuple(float(x) for x in operation.attrib["value"].split())
            if len(component) != 16:
                raise ValueError("Expected sixteen matrix elements")
        elif operation.tag in ("translate", "scale"):
            attributes(operation, {"x", "y", "z", "value"})
            default = 1. if operation.tag == "scale" else 0.
            if "value" in operation.attrib:
                if len(operation.attrib) != 1:
                    raise ValueError("Use either value or x/y/z transform attributes")
                values = tuple(float(x) for x in operation.attrib["value"].replace(",", " ").split())
                if len(values) == 1:
                    values *= 3
                if len(values) != 3:
                    raise ValueError("Expected three transform components")
            else:
                values = tuple(float(operation.get(axis, default)) for axis in "xyz")
            component = list(IDENTITY)
            for i, value in enumerate(values):
                component[4*i + (3 if operation.tag == "translate" else i)] = value
            component = tuple(component)
        elif operation.tag == "rotate":
            attributes(operation, {"x", "y", "z", "angle"})
            component = rotation(float(operation.attrib["angle"]), tuple(float(operation.get(axis, 0)) for axis in "xyz"))
        elif operation.tag == "lookat":
            attributes(operation, {"origin", "target", "up"})
            vectors = [tuple(float(v) for v in operation.attrib[key].replace(",", " ").split()) for key in ("origin", "target", "up")]
            if any(len(v) != 3 for v in vectors):
                raise ValueError("Look-at vectors need three components")
            component = look_at(*vectors)
        else:
            raise ValueError(f"Unsupported transform operation: {operation.tag}")
        # Mitsuba applies XML operations in declaration order (pre-multiplication).
        result = component if result is None else multiply(component, result)
    return result if result is not None else IDENTITY


def parse_scene(xml: bytes, name: str, *, max_depth: int | None = None) -> Scene:
    root = ET.fromstring(xml)
    attributes(root, {"version"})
    if root.tag != "scene" or root.get("version") != "3.0.0":
        raise ValueError("Expected a Mitsuba 3.0.0 scene")
    defaults = {}
    for node in root.findall("default"):
        attributes(node, {"name", "value"})
        key = node.attrib["name"]
        if key in defaults or len(node):
            raise ValueError(f"Duplicate/invalid XML default: {key}")
        defaults[key] = node.attrib["value"]

    def value(node: ET.Element) -> str:
        attributes(node, {"name", "value"})
        if len(node):
            raise ValueError(f"<{node.tag}> values cannot contain children")
        result = node.attrib["value"]
        seen = set()
        while result.startswith("$"):
            key = result[1:]
            if key in seen or key not in defaults:
                raise ValueError(f"Missing or recursive default {key!r}")
            seen.add(key)
            result = defaults[key]
        return result

    def rgb(node: ET.Element) -> tuple[float, float, float]:
        values = tuple(float(v) for v in value(node).replace(",", " ").split())
        if len(values) == 1:
            values *= 3
        if len(values) != 3:
            raise ValueError("Expected three linear RGB components")
        return values

    sensors, integrators = root.findall("sensor"), root.findall("integrator")
    if len(sensors) != 1 or len(integrators) != 1:
        raise ValueError("Expected exactly one sensor and integrator")
    sensor, integrator = sensors[0], integrators[0]
    attributes(sensor, {"type", "id"})
    attributes(integrator, {"type"})
    if sensor.get("type") != "perspective":
        raise ValueError("Only perspective source sensors are supported")
    integrator_type = integrator.attrib["type"]
    if integrator_type.startswith("$"):
        integrator_type = defaults.get(integrator_type[1:])
    if integrator_type != "path":
        raise ValueError(f"Unsupported source integrator {integrator_type!r}")
    ip = parameters(integrator, {"max_depth": "integer"})
    sp = parameters(sensor, {"fov": "float", "fov_axis": "string", "to_world": "transform", "sampler": "sampler", "film": "film"})
    film, sampler = sp["film"], sp["sampler"]
    attributes(film, {"type", "id"})
    attributes(sampler, {"type"})
    if film.get("type") != "hdrfilm" or sampler.get("type") != "independent":
        raise ValueError("Only hdrfilm and independent source sampling are supported")
    fp = parameters(film, {"width": "integer", "height": "integer", "file_format": "string", "pixel_format": "string", "rfilter": "rfilter"})
    sm = parameters(sampler, {"sample_count": "integer"})
    file_format = value(fp["file_format"]) if "file_format" in fp else "openexr"
    pixel_format = value(fp["pixel_format"]) if "pixel_format" in fp else "rgb"
    if file_format != "openexr" or pixel_format != "rgb":
        raise ValueError("Only RGB OpenEXR source film metadata is supported")
    attributes(fp["rfilter"], {"type"})
    if len(fp["rfilter"]):
        raise ValueError("Parameterized source filters are not supported")
    source_depth = int(value(ip["max_depth"])) if "max_depth" in ip else -1
    if max_depth is None and source_depth <= 0:
        raise ValueError("An unlimited source depth requires an explicit --max-depth native budget")
    render = RenderSettings(
        width=int(value(fp["width"])), height=int(value(fp["height"])),
        spp=int(value(sm["sample_count"])), max_depth=source_depth if max_depth is None else max_depth,
        filter=fp["rfilter"].attrib["type"],
    )
    source_matrix = matrix(sp["to_world"])
    camera_matrix = list(source_matrix)
    # Mitsuba camera +X points left; Gyunity camera +X points right.
    for row in range(3):
        camera_matrix[row * 4] *= -1
    fov = float(value(sp["fov"]))
    axis = value(sp["fov_axis"]) if "fov_axis" in sp else "x"
    if axis not in ("x", "y", "smaller", "larger", "diagonal"):
        raise ValueError(f"Unsupported source FOV axis: {axis}")
    effective_axis = axis
    if axis in ("smaller", "larger"):
        horizontal = (render.width < render.height) == (axis == "smaller")
        effective_axis = "x" if horizontal else "y"
    fov_y = fov
    if effective_axis == "x":
        fov_y = math.degrees(2 * math.atan(math.tan(math.radians(fov) / 2) * render.height / render.width))
    elif axis == "diagonal":
        fov_y = math.degrees(2 * math.atan(math.tan(math.radians(fov) / 2) * render.height / math.hypot(render.width, render.height)))
    scene = Scene(name=name, camera=Camera(to_world=tuple(camera_matrix), fov_y=fov_y), render=render)

    for node in root.findall("bsdf"):
        attributes(node, {"id", "type"})
        identifier = node.attrib["id"]
        two_sided = node.get("type") == "twosided"
        if two_sided:
            if len(node) != 1 or node[0].tag != "bsdf":
                raise ValueError("A two-sided BSDF must wrap exactly one BSDF")
            node = node[0]
            attributes(node, {"type"})
        kind = node.attrib["type"]
        if kind == "diffuse":
            p = parameters(node, {"reflectance": "rgb"})
            material = Diffuse(reflectance=rgb(p["reflectance"]), two_sided=two_sided)
        elif kind == "roughconductor":
            if not two_sided:
                raise ValueError("Version 1 requires two-sided conductors")
            p = parameters(node, {"alpha": "float", "distribution": "string", "specular_reflectance": "rgb", "eta": "rgb", "k": "rgb"})
            alpha = float(value(p["alpha"]))
            material = Conductor(distribution=value(p["distribution"]), alpha=(alpha, alpha),
                                 reflectance=rgb(p["specular_reflectance"]), eta=rgb(p["eta"]), k=rgb(p["k"]))
        elif kind == "dielectric" and not two_sided:
            p = parameters(node, {"int_ior": "float", "ext_ior": "float"})
            material = Dielectric(eta_inside=float(value(p["int_ior"])), eta_outside=float(value(p["ext_ior"])))
        else:
            raise ValueError(f"Unsupported source BSDF: {kind}")
        scene.add_material(identifier, material)

    for node in root.findall("shape"):
        attributes(node, {"type", "id"})
        kind = node.attrib["type"]
        allowed = {"to_world": "transform", "ref": "ref", "emitter": "emitter"}
        allowed.update({"cube": {}, "rectangle": {}, "sphere": {"radius": "float", "center": "point"},
                        "obj": {"filename": "string", "face_normals": "boolean"}}.get(kind, {}))
        p = parameters(node, allowed)
        attributes(p["ref"], {"id"})
        emission = None
        if "emitter" in p:
            emitter = p["emitter"]
            attributes(emitter, {"type"})
            if emitter.get("type") != "area":
                raise ValueError("Only source area emitters are supported")
            emission = rgb(parameters(emitter, {"radiance": "rgb"})["radiance"])
        common = dict(id=node.attrib["id"], material=p["ref"].attrib["id"],
                      to_world=matrix(p.get("to_world")), emission=emission)
        if kind == "cube":
            shape = Box(**common)
        elif kind == "rectangle":
            shape = Quad(vertices=((-1., -1., 0.), (1., -1., 0.), (1., 1., 0.), (-1., 1., 0.)), **common)
        elif kind == "sphere":
            center = p["center"]
            attributes(center, {"name", "x", "y", "z"})
            shape = Sphere(center=tuple(float(center.attrib[axis]) for axis in "xyz"), radius=float(value(p["radius"])), **common)
        elif kind == "obj":
            face_normals = value(p["face_normals"]) if "face_normals" in p else "false"
            if face_normals not in ("true", "false"):
                raise ValueError("face_normals must be true or false")
            shape = Mesh(file=value(p["filename"]).replace("/", "\\"), face_normals=face_normals == "true", **common)
        else:
            raise ValueError(f"Unsupported source shape: {kind}")
        scene.add(shape)
    for node in root:
        if node.tag not in {"default", "integrator", "sensor", "bsdf", "shape"}:
            raise ValueError(f"Unsupported source element: {node.tag}")
    scene.metadata = {"source_format": "mitsuba-3", "source_camera_to_world": list(source_matrix),
                      "source_fov": fov, "source_fov_axis": axis, "source_file_format": "openexr",
                      "source_max_depth": source_depth, "importer_version": "0.1.0"}
    scene.to_json()
    return scene


def import_archive(archive_file: Path, output_root: Path) -> Path:
    key = archive_file.stem
    if key not in ARCHIVES:
        raise ValueError(f"Unsupported gallery package: {key}")
    name, expected = ARCHIVES[key]
    digest = hashlib.sha256(archive_file.read_bytes()).hexdigest()
    if digest != expected:
        raise ValueError(f"{archive_file}: SHA-256 does not match the reviewed source archive")
    with ZipFile(archive_file) as archive:
        members = archive.infolist()
        if sum(info.file_size for info in members) > 16 * 1024 * 1024:
            raise ValueError("Gallery package exceeds the import size limit")
        for info in members:
            path = PurePosixPath(info.filename)
            if path.is_absolute() or ".." in path.parts or "\\" in info.filename or ":" in info.filename or not path.parts or path.parts[0] != key:
                raise ValueError(f"Invalid archive path: {info.filename}")
        xml = archive.read(f"{key}/scene.xml")
        license_text = archive.read(f"{key}/LICENSE.txt")
        if b"CC0" not in license_text:
            raise ValueError("Expected the reviewed CC0 license")
        scene = parse_scene(xml, name)
        scene.metadata.update(archive=archive_file.name, archive_sha256=digest,
                              xml_sha256=hashlib.sha256(xml).hexdigest(), license="CC0-1.0")
        required = {"scene.xml", "LICENSE.txt"}
        required.update(shape.file.replace("\\", "/") for shape in scene.shapes if isinstance(shape, Mesh))
        payload = {relative: archive.read(f"{key}/{relative}") for relative in sorted(required)}
    return write_package(scene, payload, output_root / key)


def write_package(scene: Scene, payload: dict[str, bytes], destination: Path) -> Path:
    for relative, data in payload.items():
        path = destination.joinpath(*PurePosixPath(relative).parts)
        if path.exists() and path.read_bytes() != data:
            raise FileExistsError(f"Refusing to overwrite different source data: {path}")
    for relative, data in payload.items():
        path = destination.joinpath(*PurePosixPath(relative).parts)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    output = destination / "scene.json"
    scene.write_json(output)
    return output


def import_xml(filename: Path, output_root: Path, name: str | None = None, *, max_depth: int | None = None) -> Path:
    """Import a supported standalone Mitsuba XML package without a pinned ZIP."""
    filename = filename.resolve()
    package = name or filename.stem
    if package in ("", ".", "..") or any(c in package for c in '<>:"/\\|?*'):
        raise ValueError("The output scene name must be a safe directory component")
    xml = filename.read_bytes()
    scene = parse_scene(xml, package, max_depth=max_depth)
    payload = {filename.name: xml}
    for shape in scene.shapes:
        if isinstance(shape, Mesh):
            relative = PurePosixPath(shape.file.replace("\\", "/"))
            asset = filename.parent.joinpath(*relative.parts).resolve()
            if not asset.is_relative_to(filename.parent):
                raise ValueError(f"Mesh escapes the source XML directory: {shape.file}")
            payload[str(relative)] = asset.read_bytes()
    license_file = filename.parent / "LICENSE.txt"
    if license_file.is_file():
        payload["LICENSE.txt"] = license_file.read_bytes()
        scene.metadata["license_file"] = "LICENSE.txt"
    scene.metadata.update(source_file=filename.name, xml_sha256=hashlib.sha256(xml).hexdigest())
    return write_package(scene, payload, output_root / package)
