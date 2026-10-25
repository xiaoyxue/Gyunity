# Gyunity
## Description
This is a physically based renderer (CPU based ray tracing), and it's original name is SmallSPPM since I have answered someone's question about sppm to write a demo. But now I decide to extend it to a graphics library, so I rename this project to "Gyunity" [dʒiːjuːnəti].

Next plan is to implement the GPU based renderer and add more features like MCMC(MMLT), Differentiable Rendering core, Path Guiding methods...

## Features
- PathTracing (PT)
- Stochastic progressive photon mapping (SPPM)
- LightTracing (LT)
- Bidirectional PathTracing (BDPT)
- Surface Vertex Connection and Merging (VCM, opt-in)
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

## Progressive surface VCM (opt-in)

`integrator\VCM.h` adds the opt-in checked renderer:

```cpp
VertexConnectionMerging vcm(sampler, maxDepth, iterations, initialRadius,
    real(2.0 / 3.0), Vcm::Mode::Combined,
    ThreadedTaskManager::DefaultConcurrency());
bool success = vcm.RenderChecked(scene, camera);
```

`RenderChecked` never saves files; `RenderScene` saves only after success.
No default example, `main()` selection, or CLI behavior changes. The callable
example below and renderer-level acceptance tests cover issue #128; bounded
tests are not a universal convergence or performance claim.

`TestVertexConnectionMerging` lives alongside the other local C++ examples in
`GyunityApp\main.cpp` (not a separately exported library function):

```cpp
bool TestVertexConnectionMerging(const std::string& sceneName = "mesh",
    const std::string& output = ExampleOutputPath("CornellBoxVcm.hdr"),
    int width = 128, int height = 128, int iterations = 16, int maxDepth = 8,
    real initialRadius = 0.05f, real alpha = real(2.0 / 3.0),
    Vcm::Mode mode = Vcm::Mode::Combined,
    SceneAccelerator accelerator = SceneAccelerator::Bvh,
    int concurrency = ThreadedTaskManager::DefaultConcurrency(),
    VertexConnectionMerging::Statistics* statistics = nullptr);
```

For example, a bounded call from that translation unit is:

```cpp
bool ok = TestVertexConnectionMerging("lucy-bunny",
    ExampleOutputPath("LucyBunnyVcm.hdr"), 32, 32, 16, 6, 0.15f,
    real(2.0 / 3.0), Vcm::Mode::Combined, SceneAccelerator::Bvh, 1);
```

All ten registered surface scenes are accepted; `fog`, unknown names and invalid
accelerator values return false. Renderer validation failures return false and
are never saved. Like the existing examples, invalid image dimensions/empty
output throw `std::invalid_argument`; missing assets and failed writes throw
`std::runtime_error` through `RenderScene`. The optional statistics pointer is
reset on entry and copied after a normal `RenderScene` return, not after an
exception. A non-null pointer is not needed to render.

Run from the worktree's `GyunityApp` directory: `..\resources` must contain the
registry's assets (`meshs\bunny3.obj`, `meshs\Alucy.obj`, `meshs\water.obj` and
`media\pictures\uffizi-large.hdr` for the representative mesh/env/water scenes).
Defaults use `Test\CornellBoxVcm.hdr` under the **main checkout**, including from
nested worktrees. Explicit output paths are unchanged and their parent directory
must exist. Tests include the actual `main.cpp` example without changing its
entry point; no `--integrator` option or PT/VPT routing change is introduced.

Start small: retained light paths and BSDF arenas scale with resolution times
depth, and the map, camera scratch, film and acceleration structures add more
memory. Increasing iterations does not increase retained path storage, but
explicit MIS and large photon neighborhoods can be costly. The modest
128x128/16/depth-8 default is not a promise that 1024x1024/1600 will fit or finish
quickly. Radius is in **scene units**, never inferred from a scene's bounds.

References: [Georgiev et al., *Light Transport Simulation with Vertex Connection
and Merging*, SIGGRAPH Asia 2012](https://iliyan.com/publications/VertexMerging/VertexMerging_SigAsia2012.pdf)
and [Georgiev, *Implementing Vertex Connection and Merging*, revision 2,
2013](https://iliyan.com/publications/ImplementingVCM/ImplementingVCM_TechRep2012_rev2.pdf).
The report's recursive `dVCM/dVC/dVM` implementation is a **test-only oracle**;
production enumerates explicit path densities. Alpha 2/3 satisfies the paper's
sufficient alpha <= 0.75 condition for its qualified asymptotic rate, not a
universal rate for all point-light/pinhole specular configurations.

Each iteration attempts exactly `P = width * height` light paths, then traces one
camera path per pixel, pairing pixel `k` with complete light path `k` for VC.
VM gathers from all **attempts**, including empty ones. The physical radius is
`R0 * pow(i + 1, -(1-alpha)/2)`, rounded once to `real`; that same radius and
`pi*R*R` are used throughout the iteration's map, MIS and normalization.
Camera contributions go through `Film::AddSample` once, including black samples,
without division by iterations. Each paired `t=1` contribution is splatted once,
scaled only by `1/iterations`. `GatherMerging` is already weighted and normalized;
the renderer adds it directly without a second MIS/kernel/photon divisor.

Stable 32x32 tiles include partial edges. Separate light/eye sampler clones per
tile persist across iterations, with seeds `2*tile + phase` and sample indices
`2*(iteration*P + pixel) + phase` (light=0, eye=1), never worker indices.
The synchronous named light phase joins before a named map-build task runs in
the same scoped concurrency model, including its nested hash-grid parallel work.
The camera phase joins before clearing references and resetting light arenas.
Separately aligned, tile-owned light arenas retain all path BSDFs until then;
eye scratch resets after each sample's consumers. Storage is `O(P*depth)`, not
proportional to iterations. Explicit enumeration remains intentionally expensive.

Modes reuse `Vcm::Mode`: Combined uses joint VC/VM MIS; ConnectionOnly skips map
construction; MergingOnly retains VM and camera-visible `LS*E` emission/background
(zero or only pure-delta interior scatters), not continuous emitter-hit overlap.
VC depth is `s+t-2`; VM depth is `s+t-3`. No `t=0` or `(1,1)` strategy is included.
Separate ConnectionOnly/MergingOnly images use different MIS denominators:
they are **not additive components** of a Combined image. Continuous
ConnectionOnly weights share BDPT's common domain; singular VCM weights follow
the report convention and need not equal legacy BDPT weights.

Requirements: initialized surface scene, valid pinhole camera/film storage, finite
positive radius, positive iterations, `0<alpha<1`, a sampler-supported depth,
concurrency `-1` or positive, and distinct non-null sampler clones returning
finite `[0,1)` values (with the bundled rounded-one conventions below). Derived radii, areas,
budgets, seeds and sample indices are checked. Scene-level checks reject unsupported
lights, media, null boundaries, duplicate emitters and rotated/multiple
environments before path storage. Every generated subpath is validated even if
no contribution consumes it, including encountered mixed lobes, endpoint PDFs,
and sampled terminal delta-event masses. No fallback or contribution clamping.
The supported surface/finite-radius limitations of the probability/map layers
below still apply, including geometric-normal shading and the map's checked
relative extent and oriented-normal filter.

Sampling-domain checks apply to the parent sampler **and each phase clone**.
The worst light budget is `max(5,3*depth+2)` draws; the eye budget is
`3*depth+2`, including jitter. Checks occur before path storage, and the
renderer-local sequence also checks every dimension/index before calling `Sample`.

| Sampler | Dimensions | Maximum renderer depth | Phase sample indices |
|---|---:|---:|---|
| `HaltonSampler` (`Halton.h`) | 256 | 84 | below `INT64_MAX` |
| `SobolSampler` | 1024 | 340 | below `2^52` (52 direction columns) |
| `RegularHaltonSampler` | 1229 | 409 | below `INT64_MAX`, so `index+1` remains signed-representable |
| Random/custom samplers | renderer cap 1229 | 409 | below `INT64_MAX` |

Opaque custom samplers must support the declared budget; no sampler framework
or legacy generator is changed. Sobol's uint32-to-float conversion and
RegularHalton's float accumulation can round to exactly one. Only these known
families retain `RandomStateSequence`'s exact-one-to-zero convention. Halton and
custom adapters returning one fail; all negative/nonfinite/greater-than-one
values fail. Other generated values are unchanged.

Valid near-one jitter is checked in double against its intended half-open pixel.
The unchanged jitter then goes to the pinhole camera, and the same rounded
`real(pixel)+jitter` goes to the film. This preserves their existing closed-edge
filter behavior when float addition rounds to `pixel+1`, including the outer film
edge; it neither clamps nor wraps the valid jitter. Finite raster/storage and
accumulated-film checks remain active.

`RequestCancellation()` cooperatively cancels an active render; all tasks drain
before failure returns. A failed film may contain partial work and must not be
saved as a successful image. `GetStatistics()` is a bounded post-render summary:
completed iterations/attempts/samples, last physical radius/area, phase counts
and times, last task-group results, and peak light-arena/path bytes and photons.
It is not process RSS and excludes map buckets, eye scratch, film and allocator
overhead. One renderer instance must not render concurrently.
Its three RGB diagnostic sums are the actual terms using the selected mode's
MIS weights: `mCameraConnectionSum` (including retained `LS*E`),
`mCameraMergingSum` (already kernel/count-normalized), and
`mSplatConnectionSum` (already divided by iterations). They do not affect film
arithmetic. For a successful fresh box-filter render with one sample per pixel
per iteration, divide camera sums by `P*iterations` and splat sums by `P` to
compare image means (allow float accumulation/RGBE quantization). They are
**pre-filter diagnostics**, not component images for arbitrary filters or
rounded boundary samples. Only interpret totals after success; failures can
contain partial work. The bounded tile-local sums are reduced after barriers.
Each VM-enabled iteration declares `2*tileCount+1` progress units (light tiles,
one map task, camera tiles); ConnectionOnly declares `2*tileCount`.
`mLastMapPhase` records the map task's scoped concurrency/status/timing.
Cancellation or failure in that phase joins and cleans up without starting eye
work or reporting overall 100%.
`mMapPhases` counts invoked groups; `mMapBuilds` counts successful constructions,
including a finished map discarded when cancellation is observed after building.

`VCMRenderer.*` exercises actual emission/black/environment/specular renders,
camera/splat normalization, radius-dependent gather arithmetic, persistent RNG
streams, partial tiles, barriers, arena reuse and single-worker determinism.
`VCMRendererNegative.*` records expected checked failures separately. Tiny HDR
checks allow RGBE quantization; the diffuse-environment check uses a separately
declared Monte Carlo/finite-radius tolerance, not a relaxed MIS tolerance.
`ExpandedEnvironmentBoundsDoNotDimDiffuseIndependentSeeds` uses 48 independent
seed-pairs, fixed geometry and doubled launch radius in all three modes. Its
central diffuse disks have no boundary bias; acceptance uses four standard
errors plus RGBE quantization and a separate precision gate. Photon-sharing
pixels are not treated as independent samples. `MainExampleRenders` calls the
real example across all registered surface scenes/modes, KD and explicit
concurrency, including no-save and filesystem-failure probes.
Run from `GyunityApp` with `GYUNITY_TEST_OUTPUT_DIRECTORY` pointing to an existing
main-checkout `Test` directory. No image-hash comparison to PT is an oracle.
Finite-radius bias, the map's normal-dot cutoff 0.9 and extent limit of 2^20
radii remain explicit approximations/limits. There is no media, mixed
delta/continuous BSDF, shading-normal adjoint, new camera, rotated/multiple
environment, adaptive-radius, independent photon-budget or GPU support.

The renderer exposed an environment PDF round-trip failure near the poles:
`acos(y)` lost latitude after float rounding. Inverse mapping now uses
`atan2(sqrt(x*x+z*z),y)` and the transverse length for the PDF Jacobian.
Emission sampling and its returned density are unchanged; the existing VCM
endpoint-consistency threshold is not relaxed.

## VCM photon map and local estimator
`integrator\VCMPhotonMap.h` provides `Vcm::PhotonMap` for issue #128.
It is **not a VCM renderer or merge-radiance estimator** and has no dependency
on VCM MIS. `Build(lightPaths, radius)` borrows a
`const std::vector<std::vector<PathVertex>>&`; complete light paths are stored
once by the caller. The index stores only radius-relative positions and
`PhotonRecord { mLightPathIndex, mVertexIndex }` IDs, never copied prefixes,
BSDF ownership, or mutable photon deposits.

- Keep the paths, their BSDF arenas, and referenced scene objects alive and
  immutable until every query finishes. Do not resize/move the paths or reset
  arenas. `Build`, `Clear`, and destruction must not race queries. Returned IDs
  refer to that generation only.
- Every valid `Surface` with a non-delta `BSDF::IsDelta()` is indexed, including
  the first surface and emitting scattering surfaces. Camera/light/environment
  **endpoint types** are skipped. Sampled and unsampled terminal pure-delta
  surfaces are excluded. Missing/empty BSDFs, mixed delta-continuous lobes,
  medium/camera surface metadata, shading-normal transport, inconsistent delta
  flags, unknown vertex types, invalid position/normal/direction/throughput or
  surface forward/reverse PDFs fail explicitly. This is not a full path-PDF or
  scene validator; the paths must come from the checked surface path builders.
- `Query(position, geometricNormal, output)` returns `bool`, clearing the
  caller-owned `std::vector<PhotonRecord>` even on failure. Use separate output
  vectors for concurrent readers; no shared mutable query scratch exists.
  Raw-bucket identities are sorted/deduplicated **before** filtering. Results
  have stable `(path, vertex)` order; distinct coincident photons stay distinct.
- Acceptance is double-precision squared Euclidean distance `<= radius^2` and
  the dot product of independently normalized, **oriented geometric normals**
  `>= 0.9`. There is no primitive-ID equality (adjacent coplanar triangles do
  not acquire seams), visibility ray, geometry term, or BSDF evaluation.
  Finite-radius neighborhoods remain approximate near boundaries/nearby
  surfaces; normal compatibility is not a general visibility solution.
  `GatherMerging` checks camera-surface eligibility separately.
- The physical radius is never clamped or rescaled. Only the broad phase uses
  `(position - firstPhotonPosition) / radius`, computed in double. Each indexed
  component must lie within `[-2^20, 2^20]`; larger extents fail, not silently
  lose photons. HashGrid receives explicit **broad-phase** radius 2 and cell
  width 4. Within these bounds float coordinate rounding is at most 1/8;
  every physical radius-one candidate is therefore inside the expanded box.
  Monotone backend arithmetic preserves its cell inclusion. Loose query bounds
  are checked in double before casts, so distant queries return an empty result
  without unsafe backend indexing. All backend cells are nonnegative and below
  `2^20`; counts are checked against `INT_MAX` and vector allocation ranges
  before building.
- Positions and normals must be finite, with components no larger than
  `sqrt(DBL_MAX)/4` in magnitude. Geometric-normal squared length and the
  finite positive radius's square must be normal finite doubles. These checks
  support the entire finite float position/radius range, including subnormal
  radii and huge translations where the relative extent permits them. Distance
  subtraction/squaring uses double, not float. Unsupported numeric ranges fail
  explicitly; the map does not change the physical kernel to accommodate them.
- Every build clears the previous points, radii, and buckets first. A failed
  checked build leaves `IsBuilt()==false`, `RecordCount()==0`, `Radius()==0`
  and queries failing; a valid empty build succeeds without querying HashGrid.
  Invalid input logs through `Error(message, false)` and returns false.
  As elsewhere in the renderer, allocation failures are not converted into
  status codes.

`RecordCount()` is **not** the attempted light-path count. No photon-count,
kernel-area, radiance, or MIS normalization is performed here. The shared
HashGrid change only adds const bucket access and defined uint32-wrap hashing;
its legacy mutable API, bucket multiplicity, and LT/SPPM behavior are retained.

`VCMPhotonMap.*` compares queries to an independent exhaustive oracle, including
boundary/nextafter, collisions, rebuild failures, extreme numbers, generated
path immutability and eight simultaneous readers. `HashGrid.*` checks independent
uint32 wrapping and legacy raw-bucket multiplicity. Run these with
`GyunityTest.exe --gtest_filter=VCMPhotonMap.*:HashGrid.*` from `GyunityApp`;
the regular framework also contains the small SPPM/scene render smoke tests.

### Local VM contribution and gather (not a renderer)
`integrator\VCMPath.h` now also provides checked `Vcm::EvaluateMerging` and
`Vcm::GatherMerging`. Both return `bool` and clear the output on failure. Errors
use `Error(message, false)`, not exception control flow or contribution clamping.

- `EvaluateMerging(scene, pathContext, &contribution)` requires a selected VM
  strategy with `s >= 2`, `t >= 2`, valid **selected prefixes**, continuous
  scattering endpoints within the physical radius, and the map's oriented
  geometric-normal compatibility test. Off-radius/incompatible/pure-delta direct
  calls fail; this low-level API does not perform a search or enforce a renderer's
  depth budget. `PathContext::lightPathCount` is the positive **attempted emitted
  path count**, including empty/missed attempts, supplied by the caller.
- `GatherMerging(scene, map, lightPaths, cameraPath, cameraPrefixCount, maxDepth,
  mode, &contribution)` queries **one** camera vertex. Call it at every eligible
  camera prefix, not just the first hit. The map must have been built from that
  exact immutable `lightPaths` object (`PhotonMap::References` checks identity).
  It derives `N_L = lightPaths.size()` after range/zero checks. Keep a complete
  per-attempt vector; do not remove empty/missed paths. A valid map containing
  no photons but a positive number of attempts returns zero. A zero-attempt batch
  is not an estimate. Neither record count nor query-hit count is a denominator.
- Gather selects `s = photon.mVertexIndex + 1`, `t = cameraPrefixCount`, and keeps
  physical depth `s + t - 3 <= maxDepth`. Depth zero has no VM; depth one includes
  first-light-surface/first-camera-surface merging. Camera origins, escapes and
  pure-delta surfaces return zero, using actual BSDF kind rather than only the
  sampled `mIsDelta` flag. Continuous surfaces after mirror/glass transport and
  environment-emitted surface photons are supported.
- **Normalization is per pair, exactly once, inside the estimator:**
  `w_VM * beta_camera * f_camera(eyeOut, originalPhotonOut) * beta_light /
  (N_L * pi * R * R)`. The weight comes from joint VCM MIS once, at that pair's
  selected counts. The camera path's **radiance-mode** BSDF is evaluated once;
  the photon BSDF is not evaluated for the local scatter. Throughputs already
  include sampling/emission compensation. There is no extra cosine, full-path
  PDF division, light-selection/emission factor, inverse-square factor or
  visibility ray between the distinct endpoints. Gather only **sums** these
  already normalized pairs; callers must not normalize its output again.
- Products, area/count normalization, and the sum use `double`, with a checked
  final conversion to `real`. Nonfinite or out-of-range results fail, without
  clipping. A valid zero BSDF/throughput returns zero, but never bypasses active
  input/MIS validation. `ConnectionOnly` returns zero without requiring a built
  map or inspecting paths/count/radius; mode/output and selected technique (pair)
  or nonnegative maxDepth (gather) still must be valid.
- Inputs must come from successful checked surface path builders. The map
  validates batch surfaces; MIS validates candidate selected prefixes. Camera
  prefixes are validated even with no hits. No unselected suffix is used by
  pair evaluation, and no positions/directions/throughputs/metadata are changed.
  Unsupported mixed lobes, media, nonzero environment rotation and malformed
  consumed metadata fail explicitly. MIS requires unit geometric-normal shading.
  Keep the scene, paths and BSDF arenas alive and immutable through all calls;
  outputs must not alias paths. Identity checking is not mutation detection:
  replacing/moving/resizing the borrowed batch in place still violates its
  lifetime contract.

`VCMEstimator.*` covers independent Lambertian and manual joint-score fixtures,
camera-side/original-direction scatter, blocked finite-radius neighborhoods,
normalization/depth/mode contracts, and a brute-force/report reference.
`VCMEstimatorGenerated.*` exercises real area/point/environment light and camera
paths, including specular transport. Expected invalid-input probes are separated
under `VCMEstimatorNegative.*`. Set `GYUNITY_TEST_OUTPUT_DIRECTORY` to an existing
main-root `Test` subdirectory for generated HDR/CSV fixtures; requested I/O failures
fail the tests explicitly. This local API does **not** establish renderer,
iteration scheduling, radius progression, film normalization or convergence
acceptance.

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
`GyunityApp\main.cpp` keeps six callable examples. With no arguments,
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
| `TestVertexConnectionMerging` | Any non-volume scene; default `mesh` | 128x128, 16 iterations, depth 8, radius 0.05, alpha 2/3, Combined | `Test\CornellBoxVcm.hdr` |

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

#### Preparatory VCM probability layer (feature branch, milestones 2-3)

`integrator\VCMPath.h` adds `Vcm::PathContext`, `Strategy`, `ComputePathPdf`, and
`ComputeMisWeight`. This is **not a VCM renderer** and changes no example, CLI,
default integrator, BDPT MIS, RNG, or film behavior. The non-owning context keeps
immutable selected light/eye prefixes; the paths and BSDF arenas must outlive it.
Area/point origins, one zero-rotation environment, the existing pinhole
convention, continuous surfaces, and pure mirror/glass chains are supported.
Mixed delta/continuous lobes, media, new camera models, rotated/multiple
environments, and malformed endpoint/event metadata fail explicitly.

The evaluator independently enumerates VC at depth `s+t-2` and VM at depth
`s+t-3`, excluding `t=0` and `(VC,1,1)`. Origin samples are reused, not resampled;
their stored forward probability already includes light selection exactly once.
For an emitter hit, the deterministic origin query includes selection once.
An unchanged reverse density is reused only when its source and sampled successor
both remain inside the selected prefix. The builder's cached value preserves the
original offset-ray directions; reconstructing those directions from hit positions
can change even a continuous-path weight. The two join-adjacent reverse densities
are still recomputed, never read from unused suffixes or stale terminal caches.
No roulette is introduced. `ComputePathPdf` returns the effective common
reduced-measure path density in logarithmic form (including `A=pi*R^2` for VM,
not its sample count). For continuous paths this is the ordinary density;
absolute singular density scales are not an additional validation oracle.
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
Merging-only retains VM plus camera-visible `LS*E` emission/background at unit
weight: **every** interior scattering event in the prefix must be pure delta.
An emitter hit after any continuous event is not covered by that exemption.
No local merge estimator, progressive radius schedule, or render loop is added
by this probability layer.

Point origins are discrete position masses; their light-selection probability
is included exactly once, and camera-hit-emitter alternatives are impossible.
An environment origin instead uses light selection times map directional PDF.
Its first surface arrival uses the projected launch-disk density, without
helper-distance falloff; reverse arrivals at infinity remain directional.
The shared checked emission APIs validate initialized bounds, and emitted rays
travel opposite the map direction. Sampled light-origin `mPdfPos`, `mPdfDir`,
`mLightSelectionPdf`, and `mPdfFwd` are checked against point/environment
endpoint metadata. Camera escapes instead query the origin probability and
use `mDirection`, not `mIsect.mOutDir`; their unused emission fields may be zero.
Changing the launch disk changes photon arrival frequency (and some weights),
not the reciprocal emitted-throughput/arrival-density normalization.

`VCMPathTest.cpp` compares every normalized strategy against **two test-only
computations**: the report's `dVCM/dVC/dVM` initialization/update/final equations
(31)-(55), and direct Eq. (10)-(12) products (binary-exponent products for long
paths). The recurrence helper is linked only into `GyunityTest`; production
never calls it or the old BDPT weight. Its geometry conversions are independent
of the production helpers. Endpoint reuse sets `p_connect=p_trace`. To align
the report's conditional-pixel camera density with Gyunity's global-film
density, the reference supplies `q_pixel=P*q_film` and `n_light=P`, with
`P=N_L`; these factors cancel in Eq. (31) and (46). When testing a different
`N_L`, both reference inputs change together, while `eta=N_L*A` changes
independently in the VC/VM competition. This is a density-convention comparison,
not an image comparison or a new arbitrary render-budget contract.

Run `GyunityTest.exe --gtest_filter="VCM*"` from the worktree's `GyunityApp`.
With `GYUNITY_TEST_OUTPUT_DIRECTORY` pointing to an existing directory under
the main checkout's `Test`, tests also write per-strategy
`vcm-weights-debug.csv` / `vcm-weights-release.csv`. The exact depth-one fixture,
noncoincident directional merges, real glossy reflection/transmission, unequal
light selection, all finite endpoint families, mode/count/radius invariants,
immutability/errors, and underflowing long products are covered. The positive
recurrence oracle also applies the report's point-light initialization,
directional environment measures, pure-delta masks, and singular updates.
The generated-path integration test additionally traces 16 deterministic light/
camera pairs through an enclosed diffuse area-lit scene, checks every positive
VC construction through depth five against BDPT and the independent oracles, and
requires positive coverage of all four endpoint families. Its XML properties
record coverage and original-versus-reconstructed direction differences.
Reporting is optional when the output variable is unset; when configured,
failure to open or flush the requested CSV fails the test with a diagnostic.

##### Approved singular convention: report weights, unchanged standalone BDPT

The user selected **report-keep-bdpt**. `VCMSingularConvention.*` preserves the
reproducible difference between legacy BDPT's delta-marker remapping and the
report's Section 5.4, Eqs. (51)-(55), as a convention regression, not a blocker.
Per-path old/new weight equality is required only for continuous paths.
The singular VCM weights are independently checked against the report and
direct/hand calculations; the legacy numbers are labelled diagnostics.
Different normalized weights alone do not establish bias in either renderer.

For an interior mirror at index `i`, the report sets the directional mass to
one, **not the converted arrival factor**: the reduced factors are
`f[i+1]=g_forward[i+1]` and `r[i-1]=g_reverse[i-1]`. Adjacent connections and
merging at the mirror are masked out. Legacy BDPT instead remaps both
delta-generated zero arrival markers to one. A common multiplicative scale
cannot reconcile these conventions when their geometric factors differ.

The exact `L-D-S-D-E` fixture uses positions `(-3,0,3), (-3,0,4), (0,0,0),
(6,0,8), (6,0,7)`, a mirror normal `(0,0,1)`, and adjoining distances 5 and 10.
Both mirror angles have cosine `4/5`. With the fixture's rational continuous
densities, feasible VC splits `s=0,1,4` have report weights **`(1,4,2)/7`**,
but legacy weights **`(1,4,8)/13`**. The report recurrence agrees with separate
direct products and hand-computed expectations, also in combined and
merging-only modes. Both VC weight sets normalize; that alone is not agreement.

Three real `GenerateLightPath`/`GenerateCameraPath` pairs with ordinary diffuse
surfaces, a mirror, an area emitter, and a pinhole camera reproduce the conflict
on nine positive emitter-hit/origin-connection/camera-splat constructions.
Changing the two mirror-edge distance ratio to `1.5, 2, 3` changes the
legacy/report ratio-of-ratios to `2.25, 4, 9`. These are depth-three probes,
not full singular VCM coverage. Separate BSDF probes cover oblique dielectric
reflection/refraction, TIR, reversed sampling, radiance/importance transport,
and one/two matching lobes without treating ordinary `Pdf()==0` as a mass.

The compatibility CSV files retain their names
`vcm-singular-gate-debug.csv` / `vcm-singular-gate-release.csv`; XML properties
now identify the **resolved** report/legacy convention. No legacy-MIS fallback
or renderer is introduced.

`BSDF::Sample` has an optional reverse-delta-mass output. The selected BxDF
provides that event's reverse component probability before its identity is
lost; both masses include the uniform matching-lobe selection probability.
Mirror returns mass one; glass uses the actual Fresnel reflection or
transmission probability in each direction, including TIR. The previous
`0.25+0.5*Re` line in glass sampling is still overridden by `P=Re`; the sampling
policy has **not** changed. Eta-squared remains in radiance throughput only.
`PathVertex::mDeltaPdf/mDeltaReversePdf` retain these successful sampled
continuations. `mPdfFwd/mPdfPrev` still contain the original BDPT zero markers,
and no origin-only probability or direction field is repurposed.

For every supported internal delta event, all feasible techniques contain the
same singular constraint exactly once. Explicit enumeration removes that
common distribution but retains its orientation's sampled component mass and
the report geometry. Equal forward/reverse component masses could cancel
across all scores; production keeps them, so it never assumes such equality.
VM support is masked independently: it is not an infeasible VC PDF multiplied
by disk area. Point `L-S-D-S-E` caustics with one eligible merge have weight one.
Camera, light, and environment records and pure-delta surfaces cannot merge.
A continuous surface reached through specular transport can merge. The
actual endpoint BSDF determines eligibility even if it has never sampled a
continuation. Conversely, emission/camera roles and unused continuations do
not invent internal scattering events.

Only consumed metadata is required. A missing forward event, unavailable
reverse mass (a custom delta BxDF without the optional event query), or invalid
mass fails checked VCM evaluation; standalone BDPT still uses its original
behavior. These fields are not a generic mixture/material framework.

The extended matrix also traces 64 real mixed-origin pairs through
diffuse/mirror/glass surfaces: 147 positive VC constructions (118 singular)
over depths 0-6, five noncoincident same-surface VM constructions after
specular transport, up to four delta events per subpath, and environment
escapes using their actual ray directions. Independent reverse-event
reconstruction does not consume the stored delta masses or reverse densities.
Additional generated oblique glass reflection/refraction, TIR, and emissive
mirror cases verify `LS*E` weights and sampling-compensated emission values.
These probability/primary-estimate tests are separate from the progressive
renderer acceptance above and do not alone prove convergence or radiance.

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
