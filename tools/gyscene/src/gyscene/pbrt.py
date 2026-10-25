"""A strict static, RGB, surface-scene PBRT v4 adapter; no renderer execution."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import gzip
import hashlib
import io
import json
import math
from pathlib import Path
import re

from .model import Camera, Conductor, Dielectric, Diffuse, IDENTITY, Mesh, RenderSettings, Scene, Sphere
from .transforms import cross, inverse, look_at, multiply, rotation

_TOKEN = re.compile(r'\s+|#[^\n]*|"(?:\\.|[^"\\])*"|\[|\]|[^\s\[\]"#]+')


@dataclass(frozen=True)
class Token:
    text: str
    quoted: bool
    location: str


def tokenize(text: str, filename: str) -> list[Token]:
    result, position, line = [], 0, 1
    for match in _TOKEN.finditer(text):
        if match.start() != position:
            raise ValueError(f"{filename}:{line}: malformed quoted string")
        raw = match.group()
        if not raw.isspace() and not raw.startswith("#"):
            result.append(Token(json.loads(raw) if raw.startswith('"') else raw,
                                raw.startswith('"'), f"{filename}:{line}"))
        line += raw.count("\n")
        position = match.end()
    if position != len(text):
        raise ValueError(f"{filename}:{line}: incomplete token")
    return result


class Tokens:
    def __init__(self, tokens):
        self.tokens, self.position = tokens, 0

    def peek(self):
        return self.tokens[self.position] if self.position < len(self.tokens) else None

    def take(self):
        token = self.peek()
        if token is None:
            raise ValueError("Unexpected end of PBRT input")
        self.position += 1
        return token

    def string(self):
        token = self.take()
        if not token.quoted:
            raise ValueError(f"{token.location}: expected a quoted string")
        return token.text

    def numbers(self, count, bracketed=False):
        if bracketed and self.take().text != "[":
            raise ValueError("Expected a bracketed transformation matrix")
        tokens = [self.take() for _ in range(count)]
        if any(t.quoted for t in tokens):
            raise ValueError("Transformation values must be unquoted numbers")
        values = [float(t.text) for t in tokens]
        if not all(math.isfinite(x) for x in values):
            raise ValueError("PBRT parameters must be finite")
        if bracketed and self.take().text != "]":
            raise ValueError("Incorrect matrix element count")
        return values

    def parameters(self):
        result = {}
        while self.peek() and self.peek().quoted:
            declaration = self.take()
            parts = declaration.text.split()
            if len(parts) != 2 or parts[1] in result:
                raise ValueError(f"{declaration.location}: invalid/duplicate parameter {declaration.text!r}")
            kind, name = parts
            values = []
            if self.peek() and self.peek().text == "[" and not self.peek().quoted:
                self.take()
                while self.peek() and (self.peek().quoted or self.peek().text != "]"):
                    values.append(self.take())
                if self.take().text != "]":
                    raise ValueError("Unterminated parameter array")
            else:
                values.append(self.take())
            if not values:
                raise ValueError(f"{declaration.location}: empty parameter {name}")
            converted = []
            for value in values:
                if kind in ("string", "texture"):
                    if not value.quoted:
                        raise ValueError(f"{value.location}: expected a string")
                    converted.append(value.text)
                elif kind == "bool":
                    if value.quoted or value.text not in ("true", "false"):
                        raise ValueError(f"{value.location}: expected unquoted true/false")
                    converted.append(value.text == "true")
                elif kind in ("float", "integer", "point2", "point3", "vector3", "normal3", "rgb"):
                    if value.quoted:
                        raise ValueError(f"{value.location}: expected a numeric parameter")
                    number = int(value.text) if kind == "integer" else float(value.text)
                    if not math.isfinite(number):
                        raise ValueError(f"{value.location}: nonfinite parameter")
                    converted.append(number)
                else:
                    raise ValueError(f"{declaration.location}: unsupported parameter type {kind}")
            result[name] = (kind, converted)
        return Params(result)


class Params:
    def __init__(self, values):
        self.values = values

    def get(self, name, kind, default=None, count=1):
        if name not in self.values:
            if default is None:
                raise ValueError(f"Missing {kind} parameter {name}")
            return default
        actual, values = self.values.pop(name)
        if actual != kind or (count is not None and len(values) != count):
            raise ValueError(f"{name}: expected {kind} with {count} value(s)")
        return values[0] if count == 1 else tuple(values)

    def finish(self):
        if self.values:
            raise ValueError(f"Unsupported parameters: {', '.join(self.values)}")


@dataclass
class State:
    transform: tuple = IDENTITY
    material: str = "default"
    emission: tuple | None = None
    reverse: bool = False
    named: dict = field(default_factory=dict)


class Importer:
    def __init__(self, filename: Path, name: str, rgb_radiance_scale: float | None):
        if rgb_radiance_scale is not None and (not math.isfinite(rgb_radiance_scale) or rgb_radiance_scale <= 0):
            raise ValueError("RGB radiance scale must be positive and finite")
        self.rgb_radiance_scale = rgb_radiance_scale
        self.root = filename.resolve().parent
        self.scene = Scene(name=name, camera=Camera(to_world=IDENTITY, fov_y=90.),
                           render=RenderSettings(width=1280, height=720, spp=16, max_depth=5, seed=0))
        self.scene.add_material("default", Diffuse(reflectance=(0.5, 0.5, 0.5)))
        self.state, self.stack = State(), []
        self.coordinates = {}
        self.sources, self.meshes = {}, {}
        self.active_files = set()
        self.world = False
        self.options = set()
        self.camera_matrix, self.camera_fov = IDENTITY, 90.

    def path(self, filename):
        path = Path(filename.replace("\\", "/"))
        path = (self.root / path).resolve() if not path.is_absolute() else path.resolve()
        if not path.is_relative_to(self.root):
            raise ValueError(f"Source path escapes the initial scene directory: {filename}")
        return path

    def read(self, path):
        raw = path.read_bytes()
        if len(raw) > 64 * 1024 * 1024:
            raise ValueError(f"Source file exceeds the 64 MiB importer limit: {path.name}")
        self.sources[path.relative_to(self.root).as_posix()] = raw
        if path.suffix == ".gz":
            with gzip.GzipFile(fileobj=io.BytesIO(raw)) as stream:
                raw = stream.read(64 * 1024 * 1024 + 1)
            if len(raw) > 64 * 1024 * 1024:
                raise ValueError("Decompressed PBRT input exceeds 64 MiB")
        return raw

    def material(self, kind, p):
        if kind == "diffuse":
            material = Diffuse(reflectance=p.get("reflectance", "rgb", (0.5, 0.5, 0.5), 3))
        elif kind in ("dielectric", "conductor"):
            roughness = p.get("roughness", "float", 0.)
            u = p.get("uroughness", "float", roughness)
            v = p.get("vroughness", "float", roughness)
            remap = p.get("remaproughness", "bool", True)
            if u < 0 or v < 0:
                raise ValueError("Roughness cannot be negative")
            if kind == "dielectric":
                if u != 0 or v != 0:
                    raise ValueError("Rough dielectric is not yet in the native model")
                material = Dielectric(eta_inside=p.get("eta", "float", 1.5))
            else:
                if u == 0 or v == 0:
                    raise ValueError("Specify positive conductor roughness; ideal conductor is not yet supported")
                # PBRT v4, unlike v3, maps roughness to alpha with sqrt().
                alpha = (math.sqrt(u), math.sqrt(v)) if remap else (u, v)
                material = Conductor(distribution="ggx", alpha=alpha,
                                     eta=p.get("eta", "rgb", count=3), k=p.get("k", "rgb", count=3))
        else:
            raise ValueError(f"Unsupported PBRT material: {kind}")
        p.finish()
        identifier = f"material-{len(self.scene.materials):04d}"
        self.scene.add_material(identifier, material)
        return identifier

    def mesh(self, identifier, positions, indices, normals=(), uv=()):
        if len(positions) % 3 or len(indices) % 3 or not indices:
            raise ValueError("Triangle meshes need point triples and index triples")
        vertices = [positions[i:i+3] for i in range(0, len(positions), 3)]
        if (normals and len(normals) != len(positions)) or (uv and len(uv) != len(vertices) * 2):
            raise ValueError("Mesh normal/UV counts must match vertex count")
        if any(i < 0 or i >= len(vertices) for i in indices):
            raise ValueError("Triangle index is outside the vertex array")
        lines = ["# Converted from PBRT source data; original source is retained."]
        lines.extend("v " + " ".join(format(x, ".17g") for x in vertex) for vertex in vertices)
        for i in range(0, len(normals), 3):
            n = normals[i:i+3]
            if sum(x*x for x in n) == 0:
                raise ValueError("Vertex normal cannot be zero")
            lines.append("vn " + " ".join(format(-x if self.state.reverse else x, ".17g") for x in n))
        for i in range(0, len(uv), 2):
            lines.append("vt " + " ".join(format(x, ".17g") for x in uv[i:i+2]))
        for i in range(0, len(indices), 3):
            face = list(indices[i:i+3])
            a, b, c = (vertices[index] for index in face)
            if sum(x*x for x in cross(tuple(y-x for x, y in zip(a, b)), tuple(y-x for x, y in zip(a, c)))) == 0:
                raise ValueError("Degenerate source triangle")
            if self.state.reverse:
                face.reverse()
            def index(n):
                n += 1
                return f"{n}/{n if uv else ''}/{n}" if normals else (f"{n}/{n}" if uv else str(n))
            lines.append("f " + " ".join(index(n) for n in face))
        relative = f"meshes\\{identifier}.obj"
        self.meshes[relative] = ("\n".join(lines) + "\n").encode("utf-8")
        return Mesh(id=identifier, file=relative, material=self.state.material,
                    to_world=self.state.transform, emission=self.state.emission)

    def shape(self, kind, p):
        identifier = f"shape-{len(self.scene.shapes):04d}"
        if p.get("alpha", "float", 1.) != 1:
            raise ValueError("Alpha-masked shapes are not supported")
        if kind == "sphere":
            radius = p.get("radius", "float", 1.)
            if p.get("zmin", "float", -radius) != -radius or p.get("zmax", "float", radius) != radius or p.get("phimax", "float", 360.) != 360:
                raise ValueError("Clipped spheres are not supported")
            m = self.state.transform
            axes = [m[j:12:4] for j in range(3)]
            scales = [math.sqrt(sum(x*x for x in axis)) for axis in axes]
            if self.state.reverse or min(scales) <= 0 or max(scales)-min(scales) > max(scales)*1e-6:
                raise ValueError("Sphere requires a positive uniform transform and outward orientation")
            for i in range(3):
                for j in range(i):
                    if abs(sum(a*b for a, b in zip(axes[i], axes[j]))) > scales[i]*scales[j]*1e-6:
                        raise ValueError("Sheared spheres are not supported")
            if sum(a*b for a, b in zip(cross(axes[0], axes[1]), axes[2])) <= 0:
                raise ValueError("Reflected sphere transforms are not supported")
            result = Sphere(id=identifier, center=(m[3], m[7], m[11]), radius=radius*scales[0],
                            material=self.state.material, emission=self.state.emission)
        elif kind == "trianglemesh":
            positions = p.get("P", "point3", count=None)
            indices = p.get("indices", "integer", (0, 1, 2) if len(positions) == 9 else None, count=None)
            result = self.mesh(identifier, positions, indices, p.get("N", "normal3", (), count=None),
                               p.get("uv", "point2", (), count=None))
        elif kind == "plymesh":
            path = self.path(p.get("filename", "string"))
            try:
                from plyfile import PlyData
            except ImportError as error:
                raise ValueError("PLY import requires uv run --extra pbrt ...") from error
            ply = PlyData.read(io.BytesIO(self.read(path)))
            vertices = ply["vertex"]
            names = set(vertices.data.dtype.names)
            if names - {"x", "y", "z", "nx", "ny", "nz", "u", "v"}:
                raise ValueError("Unsupported PLY vertex properties")
            positions = tuple(float(v[k]) for v in vertices for k in ("x", "y", "z"))
            normals = tuple(float(v[k]) for v in vertices for k in ("nx", "ny", "nz")) if {"nx", "ny", "nz"} <= names else ()
            uv = tuple(float(v[k]) for v in vertices for k in ("u", "v")) if {"u", "v"} <= names else ()
            if (names & {"nx", "ny", "nz"} and not normals) or (names & {"u", "v"} and not uv):
                raise ValueError("Incomplete PLY normal or UV properties")
            face_names = set(ply["face"].data.dtype.names)
            if face_names not in ({"vertex_indices"}, {"vertex_index"}):
                raise ValueError("Unsupported PLY face properties")
            key = next(iter(face_names))
            faces = [face[key] for face in ply["face"]]
            if any(len(face) != 3 for face in faces):
                raise ValueError("Only triangular PLY faces are supported")
            if not all(math.isfinite(v) for v in (*positions, *normals, *uv)):
                raise ValueError("Nonfinite PLY vertex data")
            result = self.mesh(identifier, positions, tuple(int(i) for face in faces for i in face), normals, uv)
        else:
            raise ValueError(f"Unsupported PBRT shape: {kind}")
        p.finish()
        self.scene.add(result)

    def parse(self, path):
        path = path.resolve()
        if path in self.active_files:
            raise ValueError(f"Include cycle: {path.name}")
        self.active_files.add(path)
        stream = Tokens(tokenize(self.read(path).decode("utf-8-sig"), str(path)))
        while stream.peek():
            token = stream.take()
            if token.quoted:
                raise ValueError(f"{token.location}: expected a directive")
            try:
                self.directive(token.text, stream)
            except (ValueError, KeyError, OSError) as error:
                raise ValueError(f"{token.location}: {error}") from error
        self.active_files.remove(path)

    def directive(self, command, s):
        if command == "Include":
            self.parse(self.path(s.string()))
        elif command == "Identity":
            self.state.transform = IDENTITY
        elif command in ("Transform", "ConcatTransform"):
            raw = s.numbers(16, bracketed=True)
            matrix = tuple(raw[4*c+r] for r in range(4) for c in range(4))
            inverse(matrix)
            self.state.transform = matrix if command == "Transform" else multiply(self.state.transform, matrix)
        elif command in ("Translate", "Scale", "Rotate", "LookAt"):
            count = {"Translate": 3, "Scale": 3, "Rotate": 4, "LookAt": 9}[command]
            v = s.numbers(count)
            if command == "LookAt":
                matrix = inverse(look_at(tuple(v[:3]), tuple(v[3:6]), tuple(v[6:])))
            elif command == "Rotate":
                matrix = rotation(v[0], v[1:])
            else:
                matrix = list(IDENTITY)
                for i in range(3):
                    matrix[4*i + (3 if command == "Translate" else i)] = v[i]
                matrix = tuple(matrix)
                inverse(matrix)
            self.state.transform = multiply(self.state.transform, matrix)
        elif command == "CoordinateSystem":
            self.coordinates[s.string()] = self.state.transform
        elif command == "CoordSysTransform":
            self.state.transform = self.coordinates[s.string()]
        elif command == "ColorSpace":
            if s.string() != "srgb":
                raise ValueError("Only linear RGB in the sRGB color space is supported")
        elif command == "WorldBegin":
            if self.world:
                raise ValueError("Duplicate WorldBegin")
            self.world = True
            self.state.transform = IDENTITY
            self.coordinates["world"] = IDENTITY
        elif command in ("Camera", "Film", "Sampler", "Integrator", "PixelFilter", "Accelerator"):
            if self.world or command in self.options:
                raise ValueError(f"Misplaced or duplicate {command}")
            self.options.add(command)
            kind, p = s.string(), s.parameters()
            r = self.scene.render
            if command == "Camera":
                if kind != "perspective" or p.get("lensradius", "float", 0.) != 0:
                    raise ValueError("Only perspective pinhole cameras are supported")
                p.get("focaldistance", "float", 1e30)
                self.camera_fov = p.get("fov", "float", 90.)
                self.camera_matrix = inverse(self.state.transform)
                self.coordinates["camera"] = self.camera_matrix
            elif command == "Film":
                if kind != "rgb":
                    raise ValueError("Only RGB film is supported")
                r.width = p.get("xresolution", "integer", 1280)
                r.height = p.get("yresolution", "integer", 720)
                self.scene.metadata["source_output_filename"] = p.get("filename", "string", "pbrt.exr")
            elif command == "Sampler":
                if kind != "independent":
                    raise ValueError("Only the independent source sampler is supported")
                r.spp, r.seed = p.get("pixelsamples", "integer", 16), p.get("seed", "integer", 0)
            elif command == "Integrator":
                if kind not in ("path", "bdpt"):
                    raise ValueError("Only surface path/bdpt integrators are supported")
                r.integrator, r.max_depth = kind, p.get("maxdepth", "integer", 5)
            elif command == "PixelFilter":
                radius = {"box": 0.5, "triangle": 2.}.get(kind)
                if radius is None:
                    raise ValueError("Only box or triangle filters are currently supported")
                x, y = p.get("xradius", "float", radius), p.get("yradius", "float", radius)
                expected = 0.5 if kind == "box" else 1.
                if x != expected or y != expected:
                    raise ValueError(f"Native {kind} filter currently requires radius {expected}")
                r.filter = "box" if kind == "box" else "tent"
            else:
                if kind not in ("bvh", "kdtree"):
                    raise ValueError("Unsupported accelerator")
                r.accelerator = "bvh" if kind == "bvh" else "kd"
            p.finish()
        else:
            if not self.world:
                raise ValueError(f"{command} requires WorldBegin")
            if command == "AttributeBegin":
                self.stack.append(replace(self.state, named=dict(self.state.named)))
            elif command == "AttributeEnd":
                if not self.stack:
                    raise ValueError("Unmatched AttributeEnd")
                self.state = self.stack.pop()
            elif command == "ReverseOrientation":
                self.state.reverse = not self.state.reverse
            elif command == "Material":
                self.state.material = self.material(s.string(), s.parameters())
            elif command == "MakeNamedMaterial":
                name, p = s.string(), s.parameters()
                self.state.named[name] = self.material(p.get("type", "string"), p)
            elif command == "NamedMaterial":
                self.state.material = self.state.named[s.string()]
            elif command == "AreaLightSource":
                kind, p = s.string(), s.parameters()
                if kind != "diffuse" or p.get("twosided", "bool", False):
                    raise ValueError("Only one-sided diffuse area emitters are supported")
                radiance = p.get("L", "rgb", count=3)
                scale = p.get("scale", "float", 1.)
                if scale < 0 or any(v < 0 for v in radiance):
                    raise ValueError("Area-light scale/radiance cannot be negative")
                if self.rgb_radiance_scale is None:
                    raise ValueError("PBRT v4 normalizes light spectra photometrically. Explicitly choose --rgb-radiance-scale for the native RGB projection; spectral equivalence is not supported.")
                luminance = sum(v*w for v, w in zip(radiance, (0.2126, 0.7152, 0.0722)))
                self.state.emission = tuple(v / luminance * scale * self.rgb_radiance_scale for v in radiance) if luminance else (0., 0., 0.)
                p.finish()
            elif command == "Shape":
                self.shape(s.string(), s.parameters())
            else:
                raise ValueError(f"Unsupported PBRT v4 directive: {command}")


def import_file(filename: Path, output_root: Path, name: str | None = None, *, rgb_radiance_scale: float | None = None) -> Path:
    filename = filename.resolve()
    if filename.stem in (".", ".."):
        raise ValueError("Invalid scene package name")
    importer = Importer(filename, name or filename.stem, rgb_radiance_scale)
    importer.parse(filename)
    missing = {"Camera", "Sampler", "Integrator", "PixelFilter"} - importer.options
    if missing or not importer.world or importer.stack:
        raise ValueError(f"Incomplete scene or unsupported implicit defaults; missing {sorted(missing)}, unmatched scopes {len(importer.stack)}")
    r = importer.scene.render
    if r.width <= 0 or r.height <= 0:
        raise ValueError("Film resolution must be positive")
    fov_y = importer.camera_fov if r.width >= r.height else math.degrees(2*math.atan(math.tan(math.radians(importer.camera_fov)/2)*r.height/r.width))
    importer.scene.camera = Camera(to_world=importer.camera_matrix, fov_y=fov_y)
    importer.scene.metadata.update(source_format="pbrt-v4", importer_version="0.1.0",
        source_files={key: hashlib.sha256(data).hexdigest() for key, data in sorted(importer.sources.items())},
        source_fov=importer.camera_fov, source_fov_axis="smaller",
        rgb_radiance_scale=rgb_radiance_scale,
        emission_conversion="Explicit RGB photometric projection: RGB / linear-sRGB luminance * source scale * rgb_radiance_scale; not PBRT spectral/film calibration.",
        note="Native RGB rendering and sampling are not PBRT spectral rendering; unsupported options are rejected.")
    text = importer.scene.to_json()
    destination = output_root / filename.stem
    payload = dict(importer.meshes)
    payload.update({"sources\\" + key.replace("/", "\\"): data for key, data in importer.sources.items()})
    for relative, data in payload.items():
        path = destination.joinpath(*relative.split("\\"))
        if path.exists() and path.read_bytes() != data:
            raise FileExistsError(f"Refusing to overwrite different source/generated data: {path}")
    for relative, data in payload.items():
        path = destination.joinpath(*relative.split("\\"))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    output = destination / "scene.json"
    importer.scene.validate_assets(destination)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8", newline="\n")
    return output
