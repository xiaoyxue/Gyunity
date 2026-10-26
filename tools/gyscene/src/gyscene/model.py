"""The native scene contract. Importers and authored scenes use these same types."""

from __future__ import annotations

import json
import math
from pathlib import Path, PureWindowsPath
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, JsonValue, model_validator

Number = Annotated[float, Field(allow_inf_nan=False)]
Positive = Annotated[Number, Field(gt=0)]
Nonnegative = Annotated[Number, Field(ge=0)]
Reflectance = Annotated[Number, Field(ge=0, le=1)]
Roughness = Annotated[Number, Field(gt=0, le=1)]
Vec3 = tuple[Number, Number, Number]
Color = tuple[Nonnegative, Nonnegative, Nonnegative]
ReflectanceColor = tuple[Reflectance, Reflectance, Reflectance]
Matrix = Annotated[tuple[Number, ...], Field(min_length=16, max_length=16)]
IDENTITY = (1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.)


def asset_path(value: str) -> Path:
    path = PureWindowsPath(value)
    if not value or path.is_absolute() or path.drive or path.root or ".." in path.parts:
        raise ValueError(f"Asset path must be relative and cannot traverse parents: {value!r}")
    return Path(*path.parts)


def validate_matrix(matrix: Matrix) -> None:
    if tuple(matrix[12:]) != (0., 0., 0., 1.):
        raise ValueError("Transforms must be affine row-major matrices")
    a, b, c, _, d, e, f, _, g, h, i = matrix[:11]
    determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(determinant) < 1e-12:
        raise ValueError("Transform is singular")


class Model(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)


class SppmSettings(Model):
    iterations: Annotated[int, Field(gt=0, le=2147483647)] = 256
    photons_per_stage: Annotated[int, Field(gt=0, le=2147483647)] = 300000


class RenderSettings(Model):
    width: Annotated[int, Field(gt=0)] = 512
    height: Annotated[int, Field(gt=0)] = 512
    spp: Annotated[int, Field(gt=0)] = 64
    max_depth: Annotated[int, Field(gt=0)] = 8
    seed: Annotated[int, Field(ge=0, le=4294967295)] = 123
    integrator: Literal["path", "bdpt", "sppm"] = "path"
    sppm: SppmSettings | None = Field(default=None, exclude_if=lambda v: v is None)
    accelerator: Literal["bvh", "kd"] = "bvh"
    filter: Literal["box", "tent", "gaussian"] = "box"

    @model_validator(mode="after")
    def dimensions(self) -> RenderSettings:
        if self.integrator == "sppm" and self.sppm is None:
            self.sppm = SppmSettings()
        if self.width * self.height > 2147483647 // 3:
            raise ValueError("Film dimensions exceed the renderer's pixel limit")
        return self


class Camera(Model):
    type: Literal["pinhole"] = "pinhole"
    to_world: Matrix
    fov_y: Annotated[Number, Field(gt=0, lt=179)]

    @model_validator(mode="after")
    def basis(self) -> Camera:
        validate_matrix(self.to_world)
        axes = [self.to_world[j:12:4] for j in range(3)]
        for i, axis in enumerate(axes):
            if abs(sum(v * v for v in axis) - 1) > 1e-3:
                raise ValueError("Camera axes must be unit length")
            for other in axes[:i]:
                if abs(sum(a * b for a, b in zip(axis, other))) > 1e-3:
                    raise ValueError("Camera axes must be orthogonal")
        return self

    @classmethod
    def look_at(cls, position: Vec3, target: Vec3, up: Vec3 = (0., 1., 0.), fov_y: float = 45.) -> Camera:
        def normalized(v: Vec3) -> Vec3:
            length = math.sqrt(sum(x * x for x in v))
            if length == 0:
                raise ValueError("Camera direction/up vectors must be nonzero and independent")
            return tuple(x / length for x in v)

        def cross(a: Vec3, b: Vec3) -> Vec3:
            return (a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0])

        forward = normalized(tuple(t - p for t, p in zip(target, position)))
        right = normalized(cross(forward, up))
        vertical = normalized(cross(right, forward))
        matrix = tuple(v for row in zip(right, vertical, forward, position) for v in row) + (0., 0., 0., 1.)
        return cls(to_world=matrix, fov_y=fov_y)


class Checkerboard(Model):
    type: Literal["checkerboard"] = "checkerboard"
    color0: ReflectanceColor
    color1: ReflectanceColor
    scale: tuple[Positive, Positive] = (1., 1.)


class Diffuse(Model):
    type: Literal["diffuse"] = "diffuse"
    reflectance: ReflectanceColor | Checkerboard
    two_sided: bool = True


class Conductor(Model):
    type: Literal["conductor"] = "conductor"
    distribution: Literal["ggx", "beckmann"]
    alpha: tuple[Positive, Positive]
    reflectance: ReflectanceColor = (1., 1., 1.)
    eta: Color
    k: Color
    two_sided: Literal[True] = True


class Dielectric(Model):
    type: Literal["dielectric"] = "dielectric"
    eta_outside: Positive = 1.
    eta_inside: Positive = 1.5
    reflection: ReflectanceColor = (1., 1., 1.)
    transmission: ReflectanceColor = (1., 1., 1.)


class Mirror(Model):
    type: Literal["mirror"] = "mirror"
    reflection: ReflectanceColor = (1., 1., 1.)


class RoughDielectric(Model):
    type: Literal["rough_dielectric"] = "rough_dielectric"
    roughness: tuple[Roughness, Roughness]
    eta_inside: Positive = 1.5
    reflection: ReflectanceColor = (1., 1., 1.)
    transmission: ReflectanceColor = (1., 1., 1.)


class Plastic(Model):
    type: Literal["plastic"] = "plastic"
    diffuse: ReflectanceColor
    specular: ReflectanceColor = (1., 1., 1.)
    roughness: Roughness


Material = Annotated[Diffuse | Conductor | Dielectric | Mirror | RoughDielectric | Plastic, Field(discriminator="type")]


class Shape(Model):
    id: Annotated[str, Field(min_length=1)]
    material: Annotated[str, Field(min_length=1)]
    to_world: Matrix = IDENTITY
    emission: Color | None = None

    @model_validator(mode="after")
    def transform(self) -> Shape:
        validate_matrix(self.to_world)
        return self


class Quad(Shape):
    type: Literal["quad"] = "quad"
    vertices: tuple[Vec3, Vec3, Vec3, Vec3]

    @model_validator(mode="after")
    def surface(self) -> Quad:
        a, b, c, d = self.vertices
        u = tuple(x - y for x, y in zip(b, a))
        v = tuple(x - y for x, y in zip(c, a))
        normal = (u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0])
        length = math.sqrt(sum(n*n for n in normal))
        if length == 0 or abs(sum(n * (x-y) for n, x, y in zip(normal, d, a))) > length * 1e-6:
            raise ValueError("Quad vertices must form a nondegenerate planar surface")
        v = tuple(x - y for x, y in zip(d, a))
        if sum(n * t for n, t in zip(normal, (
            (c[1]-a[1])*v[2] - (c[2]-a[2])*v[1],
            (c[2]-a[2])*v[0] - (c[0]-a[0])*v[2],
            (c[0]-a[0])*v[1] - (c[1]-a[1])*v[0],
        ))) <= 0:
            raise ValueError("Quad triangles must have consistent winding and positive area")
        return self


class Box(Shape):
    type: Literal["box"] = "box"
    lower: Vec3 = (-1., -1., -1.)
    upper: Vec3 = (1., 1., 1.)

    @model_validator(mode="after")
    def bounds(self) -> Box:
        if any(a >= b for a, b in zip(self.lower, self.upper)):
            raise ValueError("Box lower bounds must be below upper bounds")
        return self


class Sphere(Shape):
    type: Literal["sphere"] = "sphere"
    center: Vec3 = (0., 0., 0.)
    radius: Positive = 1.

    @model_validator(mode="after")
    def untransformed(self) -> Sphere:
        if self.to_world != IDENTITY:
            raise ValueError("Version 1 spheres use world-space center/radius; transformed spheres are unsupported")
        return self


class Mesh(Shape):
    type: Literal["mesh"] = "mesh"
    file: Annotated[str, Field(min_length=1)]
    face_normals: bool = False

    @model_validator(mode="after")
    def path(self) -> Mesh:
        asset_path(self.file)
        return self


Geometry = Annotated[Quad | Box | Sphere | Mesh, Field(discriminator="type")]


class Environment(Model):
    type: Literal["environment"] = "environment"
    id: Annotated[str, Field(min_length=1)] = "environment"
    file: Annotated[str, Field(min_length=1)]
    scale: Positive = 1.
    to_world: Matrix = IDENTITY

    @model_validator(mode="after")
    def direction_frame(self) -> Environment:
        asset_path(self.file)
        validate_matrix(self.to_world)
        if any(self.to_world[i] != 0 for i in (3, 7, 11)):
            raise ValueError("Environment transforms contain directions only, not translation")
        axes = [self.to_world[j:12:4] for j in range(3)]
        for i, axis in enumerate(axes):
            if abs(sum(v*v for v in axis) - 1) > 1e-3:
                raise ValueError("Environment axes must be unit length")
            for other in axes[:i]:
                if abs(sum(a*b for a, b in zip(axis, other))) > 1e-3:
                    raise ValueError("Environment axes must be orthogonal")
        return self


class Scene(Model):
    schema_version: Literal[1] = 1
    name: Annotated[str, Field(min_length=1)]
    camera: Camera
    render: RenderSettings = Field(default_factory=RenderSettings)
    materials: dict[str, Material] = Field(default_factory=dict)
    shapes: list[Geometry] = Field(default_factory=list)
    lights: list[Environment] = Field(default_factory=list, max_length=1, exclude_if=lambda v: not v)
    metadata: dict[str, JsonValue] = Field(default_factory=dict)

    @model_validator(mode="after")
    def references(self) -> Scene:
        ids = [shape.id for shape in self.shapes]
        if len(ids) != len(set(ids)):
            raise ValueError("Shape IDs must be unique")
        if any(not name for name in self.materials):
            raise ValueError("Material IDs cannot be empty")
        for shape in self.shapes:
            if shape.material not in self.materials:
                raise ValueError(f"{shape.id}: unknown material {shape.material!r}")
        return self

    def add_material(self, name: str, material: Material) -> None:
        if name in self.materials:
            raise ValueError(f"Duplicate material {name!r}")
        self.materials[name] = material

    def add(self, shape: Geometry) -> None:
        self.shapes.append(shape)

    def add_light(self, light: Environment) -> None:
        self.lights.append(light)

    def validate_assets(self, directory: Path) -> None:
        for shape in self.shapes:
            if isinstance(shape, Mesh):
                path = directory / asset_path(shape.file)
                if not path.is_file():
                    raise ValueError(f"{shape.id}: missing mesh {path}")
        for light in self.lights:
            path = directory / asset_path(light.file)
            if not path.is_file():
                raise ValueError(f"{light.id}: missing environment {path}")

    def to_json(self) -> str:
        # Revalidate mutable authoring collections before writing the runtime contract.
        checked = Scene.model_validate_json(self.model_dump_json())
        if not checked.shapes or not (checked.lights or any(s.emission and any(s.emission) for s in checked.shapes)):
            raise ValueError("A renderable scene needs geometry and an area or environment emitter")
        return json.dumps(checked.model_dump(mode="json"), indent=2, sort_keys=True, allow_nan=False) + "\n"

    def write_json(self, filename: str | Path) -> None:
        path = Path(filename)
        self.validate_assets(path.parent)
        text = self.to_json()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8", newline="\n")

    @classmethod
    def read_json(cls, filename: str | Path) -> Scene:
        path = Path(filename)
        scene = cls.model_validate_json(path.read_text(encoding="utf-8"))
        scene.to_json()
        scene.validate_assets(path.parent)
        return scene
