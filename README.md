# Gyunity
## Description
This is a physically based renderer (CPU based ray tracing), and it's original name is SmallSPPM since I have answered someone's question about sppm to write a demo. But now I decide to extend it to a graphics library, so I rename this project to "Gyunity" [dʒiːjuːnəti].

Next plan is to implement the GPU based renderer and add more features like MCMC(MMLT), VCM, Differentiable Rendering core, Path Guiding methods...

## Features
- PathTracing (PT)
- Stochastic progressive photon mapping (SPPM)
- LightTracing (LT)
- Bidirectional PathTracing (BDPT)
- Volume PathTracing (VPT)
- Equi-angular sampling
- Multiple importance sampling (MIS)
- Microfacet model with GGX distribution (GGX)
- Ambient Occlusion (AO)
- Low Discrepancy Sequence (Halton, Sobol...)
- KdTree, BVH
- Fast vector and matrix calculation (SIMD-SSE)
- Simple Memory Pool

## Some Results
<div align="center">
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/bunny2.png?raw=true"/>
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/bunny.png?raw=true"/>
</div>

<br/>
<div align="center">
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/coca1.png?raw=true"/>
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/coca2.png?raw=true"/>
</div>

<br/>
<div align="center">
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/EiffelTower_GGX_RoughGlass_EnvMap.png?raw=true"/>
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/EiffelTower_Glass_EnvMap.png?raw=true"/>
</div>

<br/>
<div align="center">
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/Lucy_Bunny.png?raw=true"/>
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/water_cornell_box.png?raw=true"/>
</div>

<br/>
<div align="center">
  <img style = "width: 400px; height: 400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/bunny_envmap_scale_down_1080.gif?raw=true" loop>
  <img style = "width: 400px; height: 400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/sppm_cornell_box_area_light_scale_down_1080.gif?raw=true" loop>
</div>

## Build
Ensure CMake is on `PATH`. Visual Studio 2026 requires CMake 4.2 or newer.

1. Clone the repository: `git clone --recursive https://github.com/xiaoyxue/Gyunity.git`
2. From the repository directory, run `.\init.ps1` in Developer PowerShell for VS, or `init.cmd` in Developer Command Prompt for VS. This recreates `external\build` and builds the dependencies in Debug and Release.
3. Open `Gyunity.sln` and build the solution.

## Projects
| Project | What it is |
|---|---|
| `Gyunity` | The renderer itself, built as a static library: integrators, BSDFs and materials, lights, shapes, accelerators, samplers and math. |
| `GyunityApp` | The renderer application (`x64\<Config>\GyunityApp.exe`). Use `--scene` for registered builder scenes, or no arguments for the PathTracing example in `main.cpp`. Run from `GyunityApp` so `..\resources` paths resolve. |
| `GyunityTest` | Unit tests (googletest). |

The app keeps `main.cpp` and `SceneCommandLine.*` at its root. Builder-based
scene headers live in `GyunityApp\scenes`; the unchanged original scene header
and entry point live in `GyunityApp\legacy`. Tracked `.vcxproj.filters` mirror
these app folders in Visual Studio and group `GyunityTest`'s existing `tests`
folder and shared application source. Filters organize Solution Explorer only;
compilation and legacy exclusion remain controlled by the `.vcxproj` files.

## Scene builder
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
compilation in every configuration. Both the new `main.cpp` examples and the
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

`AddTriangle` and `AddQuad` accept an optional non-owning `MediumInterface`.
Its media must outlive the scene. `FogBunny` uses an immutable, static homogeneous
medium and preserves the original material-free front boundary; use
`VolPathTracing` to compare fog renders rather than the surface-only path tracer.

## Named scenes and rendering
`GyunityApp\scenes\SceneRegistry.h` maps names to builders, default cameras and required
resources. Names are `mesh`, `slope`, `sphere`, `env`, `triangle`, `textured`,
`empty`, `diffuse`, `water`, and `fog`. The sphere and triangle scenes use their
large-room camera; all others use the standard Cornell camera. Fog automatically
uses volume path tracing; the other scenes use surface path tracing.

```powershell
Set-Location GyunityApp
..\x64\Release\GyunityApp.exe --list-scenes
..\x64\Release\GyunityApp.exe --scene mesh --out "D:\Code\Graphics\Gyunity\Test\mesh.png"
..\x64\Release\GyunityApp.exe --scene fog --accel kd --spp 128 --width 640 --height 360 --out "D:\Code\Graphics\Gyunity\Test\fog.png"
```

Defaults are 512x512, 64 samples per pixel, depth 8, and BVH. Use `--width`,
`--height`, `--spp`, `--max-depth`, and `--accel bvh|kd` to override them.
The sampler setup matches the regression harness (`RandomSampler(123)` with
the renderer's existing per-tile cloning). `--out` is required and accepts
lowercase `.png`, `.bmp`, or `.hdr`; output directories must already exist.
Paths with spaces must be quoted. `--help` and `--list-scenes` are standalone
commands. Invalid options, missing assets and failed image writes report an
error and return a nonzero exit code; no image is silently substituted.

## C++ integrator examples
`GyunityApp\main.cpp` keeps four runnable mock examples. With no arguments,
`main()` calls `TestPathTracing()` instead of the previous LightTracing demo.
To select another C++ example, replace that call with `TestSPPM()`,
`TestLightTracing()`, or `TestVolPathTracing()` and rebuild. Explicit CLI
arguments still use `RunSceneCommandLine`; they do not select these examples.

| Function | Builder scene | Default settings | Output |
|---|---|---|---|
| `TestPathTracing` | `env` | 1024x1024, 100 spp, depth 10 | `Test\CornellBoxPtEnvmap.bmp` |
| `TestSPPM` | `mesh` | 1024x1024, 256 iterations, 300000 photons/stage, depth 20, radius 0.05, alpha 0.66666667 | `Test\CornellBoxSppmTest.bmp` |
| `TestLightTracing` | `mesh` | 1024x1024, 512 spp, depth 15 | `Test\CornellBoxLT.bmp` |
| `TestVolPathTracing` | `fog` | 1024x1024, 2000 spp, depth 8 | `Test\MediaTestFogEquiAngular.bmp` |

Default outputs always go to `Test` at the **main repository root**, not the
working directory or a worktree's root. The examples locate the main checkout
from the executable's ancestors, skipping nested worktrees' `.git` files, and
create `Test` if necessary. Keep worktrees under the main checkout's `.worktree`
directory as described in `AGENTS.md`. Default filenames have no numeric suffix.

All examples use BVH, a box filter and the registered scene camera. PathTracing,
LightTracing and VolPathTracing retain `RandomSampler(123)`; SPPM retains
Halton sampling, per-hit-point radius shrinking and glossy-ray tracing.
VolPathTracing retains its volume-specific settings and the fog builder's
long-lived medium. PT/VPT intentionally use the registry's look-at camera
constructor, not the legacy basis constructor that rebuilds its basis from
unset look-at/up vectors.

Scene construction, asset checks and checked image saving are shared with the
CLI through `RenderScene`. Run from `GyunityApp` for asset paths. Example functions
accept an output filename, width, height and sample/iteration count; SPPM also
accepts a photon budget. An explicitly supplied output path is used unchanged.
For a small render, the selected call can be:

```cpp
TestPathTracing("D:\\Code\\Graphics\\Gyunity\\Test\\example.png", 64, 64, 4);
```

The settings above intentionally preserve the representative legacy demos and
are separate from the CLI's 512x512 / 64 spp / depth-8 defaults. Missing resources
or failed image writes report an error and return a nonzero application exit.

The example smoke tests use small workloads through the same functions and
check readable, nonblack HDR outputs. Set an existing output directory under the
main checkout's `Test` to enable them; otherwise these render tests are skipped:

```powershell
# From the worktree's GyunityApp directory:
$env:GYUNITY_TEST_OUTPUT_DIRECTORY = "D:\Code\Graphics\Gyunity\Test\scene_builder_main"
..\x64\Debug\GyunityTest.exe --gtest_filter="MainExamples.*:MainExampleRenders.*"
..\x64\Release\GyunityTest.exe --gtest_filter="MainExamples.*:MainExampleRenders.*"
```

## Bidirectional probability interfaces
The first BDPT preparation step adds deterministic probability queries without
adding a selectable, incomplete BDPT renderer. Existing PT/LT paths remain in use.

- `Scene::LightSelectionPdf(lightIndex, &pdf)` evaluates the PMF of a registered
  light slot from the same distribution as `SampleOneLight`; before initialization
  both use uniform selection.
- `Light::PdfLight(point, dir, &pdfPos, &pdfDir)` evaluates conditional emitted-ray
  densities, without sampling or including the light-selection PMF. `dir` is a
  unit direction emitted by the light, matching `SampleLight`.
- Area-light position density is per unit area; point-light position is a
  **discrete mass** (one at the emitter), distinguished by `IsDeltaLight()`.
  Do not combine that mass with continuous area densities.
- Environment position density is for the auxiliary launch disk; the direction
  query evaluates the map at `-dir`. It does not depend on a finite helper
  endpoint's distance and is not a physical area-emitter density.

New queries return `bool`; unsupported or invalid queries return `false`, set
their output densities to zero and log with exceptions disabled. A successful
query may legitimately produce a zero density. Discrete sampling now handles
an exact-one input and the constructor's existing all-zero uniform-CDF case.
This does not change continuous environment-map sampling or introduce a new
exception-handling framework. `LightProbabilityTest.cpp` covers analytical PMFs,
sample/PDF agreement, hemisphere normalization, endpoint measures and failures.

## Bidirectional subpath generation
`integrator\BDPTPath.h` provides the next preparation step: `GenerateCameraPath`
and `GenerateLightPath`, using LT's path vocabulary without a second rendering
framework. No BDPT render mode is exposed until connections and MIS are complete.

Each function fills a `std::vector<PathVertex>` and returns its vertex count,
including its endpoint. `maxVertices` is a storage/extension budget, not the
eventual renderer's scattering depth. Zero produces an empty path; `-1` reports
an invalid or unsupported configuration with non-throwing logging. Discard partial
output on `-1`. Keep the supplied `MemoryPool` alive until connections are finished.

Camera paths use radiance transport and light paths use importance transport.
For BDPT generation only, shading frames are explicitly built from geometric
normals; interpolated shading-normal transport is not advertised as supported.
Media, null-material boundaries and mixed delta/continuous BSDFs are rejected.
Pure mirror/glass events remain supported. A zero-contribution sample stops
extension without losing the surface vertex already reached.

`mPdfFwd` stores the incoming density at a vertex and `mPdfPrev` stores the reverse
density at that vertex from its successor. Finite destinations use geometric-normal
area conversion. An environment endpoint stores its direction toward the map;
its origin density is light-selection PMF times directional PDF, while the first
surface density is launch-disk PDF times the projected normal, without inverse-square
falloff. Delta-event densities are zero markers, separate from a point light's
delta position. `mIsDelta` is set only when a specular scattering event was sampled.
The existing LT implementation and its use of the original fields remain unchanged.

`BDPTPathTest.cpp` checks forward/reverse densities, preservation of incoming
directions, geometric conversion, budgets, endpoint metadata, explicit failures,
glass transport modes, and environment launch/escape conventions.

## Pixel debug selection
Pixel diagnostics are local to the calling thread, including calls outside TBB.
Use `SelectDebugPixel(x, y, targetX, targetY)` or
`SelectDebugRegion(x, y, minX, minY, maxX, maxY)` (inclusive bounds), then inspect
`IsPixelDebugEnabled()` at the diagnostic site. `SetPixelDebugEnabled(bool)`
enables or clears the current thread explicitly. There is no fixed worker-count
limit and callers do not need a TBB thread index.

Legacy `GYT_DEBUG_PIXEL*` macros forward to these functions for existing snippets;
their thread-index argument is evaluated but no longer indexes shared storage.
Selection remains active on that thread until another selection or explicit reset.
The utility does not change SPPM accumulation or introduce exceptions.

## Scene regression
Run the registered-scene regression without patching or recompiling `main.cpp`:

```powershell
# From the worktree root, after building Release|x64:
.\GyunityTest\RunSceneRegression.ps1 `
  -OutputDirectory "D:\Code\Graphics\Gyunity\Test\scene_registry_step4\renders" `
  -ReferenceDirectory "D:\Code\Graphics\Gyunity\Test\remaining_scenes_step3"
```

The script enumerates `--list-scenes`, renders each scene with BVH and kd-tree
through the actual application, and compares PNG file hashes against
`old.<scene>.<accel>.png` references from the original scene harness. References
must use the same image writer, cameras and default render settings. It writes
logs, images and `comparisons.csv` only to the supplied output directory (use
`Test` at the main repository root), and fails if any render or comparison fails.
