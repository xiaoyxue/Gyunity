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

## Scoped threaded task groups
`system\ThreadedTaskManager.h` adds a small, synchronous TBB-backed preparation
API for future staged renderers. It does not replace `ParallelFor` or migrate
LT, SPPM, BDPT, their RNGs, `ThreadIndex`, `ThreadsNumber`, or `Spinlock`.
Like Yuanming's `ThreadedTaskManager`, it wraps scoped `task_arena` and
`parallel_for`, not a persistent custom pool, async queue, or dependency graph.

```cpp
bool PrepareValues(std::vector<size_t>& values, ProgressReporter& progress) {
    using Tasks = ThreadedTaskManager;
    const auto phase = Tasks::Run("Prepare values", 0, values.size(),
        [&](size_t index, const Tasks::Context& context) {
            if (context.IsCancellationRequested()) return Tasks::Status::Cancelled;
            values[index] = index;
            return Tasks::Status::Success;
        }, Tasks::DefaultConcurrency(), &progress);
    return phase.mStatus == Tasks::Status::Success;
}
```

- `Run(name, begin, end, task, maxConcurrency, progress)` processes `[begin, end)`
  using stable `size_t` task indices, including nonzero offsets. Task index, not
  the arena-local `ThreadIndex()`, is the identity for data and RNG ownership.
  There is no promised order, worker affinity, or minimum number of workers.
- The optional concurrency argument defaults to **1 in Debug**, **-1 (TBB
  automatic) in Release**. A positive maximum, or explicit `-1`, overrides either
  build's default only for this run. No process-global control is installed.
  Nested TBB work is isolated from sibling callbacks to avoid callback reentry.
- Returning is a phase barrier: all started callbacks and their progress updates
  have drained, including on failure/cancellation. Callbacks must be thread-safe,
  must not throw, and must not launch unjoined work or retain the run's context.
  Exceptions from unexpected infrastructure failures are not translated into
  task statuses. This API is not an exception-handling framework.
- A callback returns `Status::Success` for a completed unit, `Failed` for an
  explicit failure, or `Cancelled` for an abandoned unit. Either of the latter
  requests cancellation of remaining work. `Context::RequestCancellation()` is
  also thread-safe; long-running callbacks should poll `IsCancellationRequested()`.
  Cancellation is cooperative, not forced termination: a callback racing with a
  request may still start, and started callbacks must finish or cooperate.
- Group status is `Failed` if any callback fails (an invalid enum value also
  fails), otherwise `Cancelled` if cancellation was requested, otherwise
  `Success`. Failure takes precedence. Completed callbacks remain completed even
  after a cancellation request; a late request can therefore produce `Cancelled`
  with zero cancelled units. Independent runs always have fresh state.
- Empty ranges succeed, but validation still applies. A blank name, reversed
  range, null/empty callback, or limit other than a positive integer or `-1`
  logs an error without throwing and returns `Failed` before starting work.
  Callback failures are logged once per group, not per task.

`Result` contains the group name and final coarse-task statistics:
`mPlanned = end - begin`, `mStarted` counts admitted callback invocations,
`mCompleted` counts successful callbacks, and `mFailed` counts failed callbacks.
`mCancelled` includes **both abandoned started tasks and all not-started tasks**.
Thus `planned = completed + failed + cancelled`, and
`started - completed - failed` counts abandoned started tasks.
`mActive` is zero after the barrier; `mPeakConcurrency` is the peak number of live
callbacks (including completion bookkeeping). `mConcurrencyLimit` is the resolved
arena maximum. `mElapsed` is a `steady_clock` duration in seconds, including setup
and draining. Validation failures have no arena/started/failed tasks and cancel
the whole valid range; a reversed range has zero planned tasks.

The optional `ProgressReporter*` is borrowed and must outlive the run. The caller
must provide enough remaining units across all phases. Only successful callbacks
call `Advance()` once; failed, abandoned, skipped, and empty work never advances.
The manager **never calls `Finish`**, even for a successful final phase. The render
controller must stop dependent phases on non-success and call `Finish(false)`,
or `Finish(true)` only after the entire render succeeds. No per-vertex telemetry
or extra success logging is added.

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
`GyunityApp\main.cpp` keeps five runnable mock examples. With no arguments,
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
| `TestBidirectionalPathTracing` | Any non-volume scene; default `mesh` | 1024x1024, 256 spp, depth 8 | `Test\CornellBoxBdpt.bmp` |

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
BDPT uses deterministic probability queries without changing the existing PT/LT
sampling conventions.

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
framework. These helpers back `BidirectionalPathTracing`.

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

## Bidirectional path tracing
`BidirectionalPathTracing(sampler, maxDepth, spp)` supports finite area/point
lights and one environment light, alone or mixed with finite lights.
`RenderChecked(scene, camera)` returns success/failure explicitly; the shared
`RenderScene` checks this result before saving the film. Other integrators retain
their existing behavior through the default checked-render adapter.
Media, mixed delta/continuous BSDFs and multiple environment maps are explicitly
unsupported. Environment emission requires finite nonempty scene bounds.

The renderer generates both subpaths, evaluates supported connections at
`depth = s + t - 2`, adds each camera-sample sum once and normalizes camera splats
once by samples per pixel. It does not call the PT direct-light estimator.
Only geometric-normal shading is used by its path builders.

`EvaluateConnection` in `integrator\BDPT.h` returns the primary connection estimate:
**without MIS**, but still including the original sampling compensation, endpoint
density and visibility checks. It supports camera hits/escapes, reused light
origins, camera splats and surface connections without reading cached full-path
MIS densities. `ConnectPaths` remains the BDPT wrapper: skip MIS for a zero
primary, otherwise multiply by the existing `MisWeight`. Both functions keep
input paths unchanged and leave the contribution zero on failure. Raster output
retains the original camera-projection behavior, including blocked splats.

The internal `Bidirectional` namespace in `integrator\BDPTPath.h` shares direction,
light-origin density, toward-vertex density and VC-feasibility queries without
duplicating their implementations. Geometry and density conversion retain their
distinct surface predicates. This is preparation for VCM, not a VCM integrator or
a new MIS evaluator; scene validation and rendering/scheduling are unchanged.

`MisWeight` follows the original PBRT-v3 balance heuristic: walk neighboring
strategies in both directions using forward/reverse density ratios, then evaluate
`1 / (1 + sumRatios)`. Scalar accumulation uses double precision. Only temporary
connection-adjacent densities are changed in separate light/camera MIS snapshots.
These snapshots replace PBRT's `ScopedAssignment` restoration mechanism: the
input paths are const and never modified, including on early returns. Endpoint
and predecessor references and the two ratio walks follow PBRT's ordering. Ordinary
zero-density events are distinct from delta-event markers. Point-light camera-hit
strategies, specular connections, `(1,1)` and `t=0` are excluded.

The implementation reuses generated endpoints, so its conditional endpoint PDFs
are not copied from PBRT's endpoint-resampling branch. Environment connections use
the stored map direction and its selection-weighted directional PDF, not the
position of a finite helper point. Emitted environment paths use projected
launch-disk density at the first surface. Camera misses retain accumulated
throughput and evaluate the environment in the recorded direction.

Independent product tests cover diffuse paths, unequal emitter PMFs, point
sources, mirror chains and environment strategies. Analytical constant-environment
diffuse, mirror, glass and mixed point-light checks supplement the 16x16 smoke
renders. Environment rotation remains an existing unsupported map feature; use
zero rotation rather than treating PT agreement as a rotation reference.
BDPT checks the requested environment rotation and reports a nonzero value as
unsupported before rendering; the existing PT/environment implementation is
unchanged.

Connection contract tests independently check primary values and the unchanged
primary-times-BDPT-MIS result, including a nonzero primary with corrupted MIS
density: primary evaluation succeeds, while the wrapper fails with zero output.
They also cover immutable inputs, invalid counts, zero/blocked/off-film results,
delta endpoints and environment direction/launch measures.

`BDPTAcceptanceTest.cpp` directly exercises all four supported strategy families,
isolates camera splats with primary rays that miss all geometry, checks splat
normalization at multiple sample counts, expands auxiliary environment launch
bounds without changing geometry, and checks latitude/longitude signs using a
directional HDR marker map. Generated fixtures and images use
`GYUNITY_TEST_OUTPUT_DIRECTORY` under the main checkout's `Test` directory.

#### Preparatory VCM probability layer (feature branch, milestone 2)

`integrator\VCMPath.h` adds `Vcm::PathContext`, `Strategy`, `ComputePathPdf`, and
`ComputeMisWeight`. This is **not a VCM renderer** and changes no example, CLI,
default integrator, BDPT MIS, RNG, or film behavior. The non-owning context keeps
immutable selected light/eye prefixes; the paths and BSDF arenas must outlive it.
Only finite area-light origins, the existing pinhole convention, and continuous
surface scattering are currently accepted. Point/environment origins, specular
chains (including terminal delta BSDFs), mixed delta lobes, and media fail
explicitly. Milestone 3 must complete the singular/environment measures.

The evaluator independently enumerates VC at depth `s+t-2` and VM at depth
`s+t-3`, excluding `t=0` and `(VC,1,1)`. Origin samples are reused, not resampled;
their stored forward probability already includes light selection exactly once.
For an emitter hit, the deterministic origin query includes selection once.
No roulette is introduced. `ComputePathPdf` returns the raw effective path
density in logarithmic form (including `A=pi*R^2` for VM, not its sample count).
`ComputeMisWeight` uses double log-products and common-maximum subtraction to
normalize the balance scores `pVC` and `N_L*pVM`. It does **not** hide `1/N_L` in
the VM weight. `N_L` is attempted light paths, not stored photons or query hits.
Invalid input returns `false`; ordinary zero/disabled alternatives return
`true` with `feasible=false`. Read `logPdf` only when feasible. Requesting a
weight for a zero-support/disabled current strategy fails.

Finite-radius mapping follows [Implementing VCM, revision 2, Section 4.3 /
Figure 3](https://iliyan.com/publications/ImplementingVCM/ImplementingVCM_TechRep2012_rev2.pdf).
For light predecessor `a`, photon landing `y`, camera query `x`, and camera
predecessor `b`, the current VM density is `A*p_L(...,a,y)*p_E(...,b,x)`.
The density for sampling `a` backwards uses the BSDF **at x** with original
eye/photon directions, converted along `y->a`; sampling `b` forwards uses
the same camera interaction with swapped directions, converted along `x->b`.
The next reverse density at the light predecessor still sees `y`, not `x`.
The adjacent VC cuts join `a->x` and `y->b`, respectively: their connection
edges are **unsampled**, so their PDFs are just the corresponding unchanged
prefix products (one excludes the photon landing density, the other excludes
the camera-query density). Neither an `x-y` edge nor a universally collapsed
polyline is introduced. This is the report's finite-radius approximation,
not an assertion that all techniques sample one identical geometric path.

Combined and connection-only modes use their respective strategy families.
Merging-only retains VM plus depth-zero visible-emitter hits; the later
singular-path stage must extend this to `LS*E`. No photon map, local merge
estimator, progressive radius schedule, or render loop exists yet.

`VCMPathTest.cpp` compares every normalized strategy against **two test-only
computations**: the report's `dVCM/dVC/dVM` initialization/update/final equations
(31)-(47), and direct Eq. (10)-(12) products (binary-exponent products for long
paths). The recurrence helper is linked only into `GyunityTest`; production
never calls it or the old BDPT weight. Its geometry conversions are independent
of the production helpers. Endpoint reuse sets `p_connect=p_trace`. To align
the report's conditional-pixel camera density with Gyunity's global-film
density, the reference supplies `q_pixel=P*q_film` and `n_light=P`, with
`P=N_L`; these factors cancel in Eq. (31) and (46). When testing a different
`N_L`, both reference inputs change together, while `eta=N_L*A` changes
independently in the VC/VM competition. This is a density-convention comparison,
not an image comparison or a new arbitrary render-budget contract.

Run `GyunityTest.exe --gtest_filter="VCMPath.*"` from the worktree's `GyunityApp`.
With `GYUNITY_TEST_OUTPUT_DIRECTORY` pointing to an existing directory under
the main checkout's `Test`, tests also write per-strategy
`vcm-weights-debug.csv` / `vcm-weights-release.csv`. The exact depth-one fixture,
noncoincident directional merges, real glossy reflection/transmission, unequal
light selection, all finite endpoint families, mode/count/radius invariants,
immutability/errors, and underflowing long products are covered. The positive
continuous recurrence oracle does not purport to validate singular paths.

`TestBidirectionalPathTracing` in `main.cpp` accepts the scene name, output path,
width, height, spp, depth and accelerator. It covers all nine original surface
scenes plus `lucy-bunny`, rejects `fog`, and does not change the default no-argument
PT example or the CLI's existing integrator selection. To select the new example
in `main`, replace the default render call with:

```cpp
if (!TestBidirectionalPathTracing("lucy-bunny",
    ExampleOutputPath("CornellBoxBdptLucyBunny.png"))) return 1;
```

The example defaults to **1024x1024 and 256 spp**. Tests pass explicit smaller
settings. `lucy-bunny` contains rough-glass Lucy and a purple-glass bunny inside
the Cornell box, illuminated only by its ceiling area light, with no added HDR.

BDPT reports progress to stderr using the same `Percentage:` label as PT.
It starts at 0%, advances monotonically as tiles finish all their samples, and
prints 100% only after every worker succeeds. Output is throttled to 0.1 percentage
point steps instead of printing for every sample. This measures completed tiles,
not estimated time remaining; different paths can take different amounts of time.
Scene loading precedes this render progress, and image saving follows it.
Each 32x32 tile owns a sampler clone and memory pool, matching PT's scheduling
granularity. Progress is completed tiles divided by total tiles; partial edge
tiles count as one tile each. Camera splats
can land outside the tile and still use the film's synchronization.
Tile-based seeds change the random sequence from the older row-based version;
the estimator and normalization are unchanged, but image hashes need not match.
`ProgressReporter` owns counting, synchronization, throttled output and completion
state; the renderer only calls `Advance()` for each finished tile and
`Finish(success)` after joining workers. Failed renders report the completed
work-unit count without a 100% message. If stderr
is redirected to a log, monitor that log (for example with `Get-Content -Wait`)
or use `Tee-Object` to retain both console output and a log.

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

Use `-Scenes mesh,slope,sphere,env,triangle,textured,empty,diffuse,water,fog` when
comparing against the historical ten-scene references; `lucy-bunny` is new and
has no original-scene baseline. Without `-Scenes`, the runner requires references
for every registered scene rather than silently skipping missing references.

A Python equivalent is available without additional packages:

```powershell
uv run --no-project .\GyunityTest\RunSceneRegression.py `
  --output-directory "D:\Code\Graphics\Gyunity\Test\python-scene-regression" `
  --reference-directory "D:\Code\Graphics\Gyunity\Test\remaining_scenes_step3" `
  --scenes mesh slope sphere env triangle textured empty diffuse water fog
```

It uses Python 3.11 or newer and standard-library modules only. Like the PS1
version, it invokes the actual app, preserves reference files, writes per-render
logs and a CSV report, and fails on unknown scenes, missing references, render
errors or hash mismatches. Both scripts remain supported.

The script enumerates `--list-scenes`, renders each scene with BVH and kd-tree
through the actual application, and compares PNG file hashes against
`old.<scene>.<accel>.png` references from the original scene harness. References
must use the same image writer, cameras and default render settings. It writes
logs, images and `comparisons.csv` only to the supplied output directory (use
`Test` at the main repository root), and fails if any render or comparison fails.
