# Scene builder

`GyunityApp\scenes\SceneBuilder.h` provides material factories and ordered helpers for
triangles, quads, spheres, meshes, area lights and environment lights. It does not
initialize the scene or select its accelerator. Existing `legacy\CornellBox.h` scenes
remain unchanged while they are migrated in separate steps.

`GyunityApp\scenes\CornellBoxScenes.h` adds parallel, builder-based versions:

| Original class in `legacy\CornellBox.h` | New function |
|---|---|
| `CornellBoxMesh` | `CornellBoxScenes::GlassBunny` |
| `CornellBoxMeshWithSlope` | `CornellBoxScenes::Slope` |
| `CornellBoxSphere` | `CornellBoxScenes::SphereBox` |
| `EnvironmentMapScene` | `CornellBoxScenes::Environment` |
| `CornellBoxTriangle` | `CornellBoxScenes::TriangleBox` |
| `CornellBoxTriangle2` | `CornellBoxScenes::TexturedSpheres` |
| `CornellBoxMeshEmpty` | `CornellBoxScenes::Empty` |
| `CornellBoxDiffuseBunny` | `CornellBoxScenes::DiffuseBunny` |
| `CornellBoxWater` | `CornellBoxScenes::Water` |
| `CornellBoxMedium` | `CornellBoxScenes::FogBunny` |

Each new function accepts a `SceneBuilder&`. For example:

```cpp
SceneBuilder builder(scene);
CornellBoxScenes::GlassBunny(builder);
```

The original scene classes and entry-point source (`GyunityApp\legacy\main_lagcy.cpp`)
are retained for side-by-side comparison. The legacy source is excluded from
compilation in every configuration. Both the `Examples.cpp` helpers and the
`--scene` path use the builder versions.
Use the same camera, accelerator and integrator when comparing a scene pair.
`GyunityTest` compares primitive order, sampled geometry, materials and lights
between all ten pairs, including image-texture UVs and fog scattering. Run it
from `GyunityApp` to resolve the scene assets:

```powershell
Set-Location GyunityApp
..\x64\Debug\GyunityTest.exe
..\x64\Release\GyunityTest.exe
```

`AddCornellBox(builder)` adds the standard walls and ceiling light, matching
`CornellBoxMeshEmpty`. `CornellBoxOptions` can omit the back wall or light and
change the light's half-size and power multiplier (defaults: `0.25f` and `85`).
For scenes whose objects precede the light, keep that insertion order:

```cpp
SceneBuilder builder(scene);
AddCornellBoxWalls(builder);
builder.AddMesh("meshs\\bunny3.obj", builder.Glass(Vec3(1), Vec3(1)),
    Transform::Translate(Vec3(0, -0.5, 0)) * Transform::Scale(0.28, 0.28, 0.28));
AddCornellBoxLight(builder, 85);
```

Resource paths are relative to `resources`; run from `GyunityApp` as before.
Quads use triangles `(a, b, c)` and `(a, c, d)` with an explicit normal.
Use `AddTriangle` for a different vertex order: even cyclic permutations can
change floating-point results. Emitting quads register two area lights sharing
the same shapes as their corresponding primitives.

`ImageColor(path)` supplies an image texture to the `Diffuse(texture)` overload.
The transformed `AddSphere(center, radius, material, objectToWorld)` overload
retains the sphere's object-space texture mapping. `AddCornellSphereWalls`
shares the large inward-facing sphere walls used by `SphereBox` and `TriangleBox`.

For finite, nonsingular object transforms and valid surface normals, transformed
intersections use inverse-transpose normals and an orthonormal shading frame.
The projected transformed shading-u tangent determines its orientation; the
transformed shading-v tangent determines its handedness (including reflections).
If the u projection degenerates, v supplies the orientation; if both degenerate,
`CoordinateSystem` supplies a deterministic right-handed basis. Geometric
derivatives and UV coordinates are not replaced with the shading basis.

The public look-at camera keeps the existing right/up/forward projection for
nondegenerate inputs. A nonzero up vector parallel or nearly parallel to a
nonzero view direction (squared sine at most `1e-12`) produces an explicit warning
and uses `CoordinateSystem(view)`'s first perpendicular vector as its deterministic
up. No position, target, FOV or film defaults are invented.
`PinHoleCamera::Create(film, pos, lookAt, up, fovy, distance)` returns null on
invalid inputs and logs a non-throwing error. Direct constructors remain available;
check `IsValid()` before using them. Invalid cameras return zero importance/PDFs
and an empty (`tMax = 0`) ray. Explicit camera axes must be finite, unit and
orthogonal; either supplied handedness is preserved. Native JSON already checks
its camera matrix for orthonormal axes before construction.
Registered camera creation propagates checked failure. Both `RenderScene` entry
points reject null or invalid registered/native cameras before scene acceleration,
integrator execution or image writing. Failed camera reinitialization clears
`IsValid()` rather than retaining a previously successful state.

Checked `ImageIO::LoadImage` / `LoadTexture` return false and leave caller outputs
unchanged on missing or corrupt input. Native gray and gray-alpha images expand
gray into RGB, while RGB/RGBA retain RGB; alpha is ignored, not premultiplied.
For LDR inputs stb's existing gamma-2.2 decoding remains unchanged. Texture loading
flips rows; ordinary image loading does not. Decoder buffers are owned and freed
inside the loader.

`AddTriangle` and `AddQuad` accept an optional non-owning `MediumInterface`.
Its media must outlive the scene. `FogBunny` uses an immutable, static homogeneous
medium and preserves the original material-free front boundary; use
`VolPathTracing` to compare fog renders rather than the surface-only path tracer.
