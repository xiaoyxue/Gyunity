# Progressive surface VCM (opt-in)

## Reading the implementation

`RenderChecked` prepares one render-local `RenderState`, runs `RenderIteration`,
then clears each iteration's paths only after its task groups have joined.
The state owns the complete light-path batch, map and tile arenas; it is not a
new shared renderer base or mutable state retained between renders.

`RenderIteration` shows the three synchronous phases directly: `TraceLightPaths`,
`PhotonMap::Build`, and `TraceCameraPaths`. The camera loop generates one path,
then walks `t` in order, calling `Connect` followed by `Merge`. `Connect` walks
`s` and reuses BDPT's `EvaluateConnection`; `Merge` adds the already-normalized
`GatherMerging` result. There is no second BDPT tracer or primary estimator.
Counters and storage accounting are kept outside the contribution equations.

In `VCMPath.cpp`, `ForwardDensity` and `ReverseDensity` follow the light and
camera sampling directions explicitly. `LightVertex` and `CameraVertex` retain
the different physical endpoints of a finite-radius merge. `StrategyDensity`
forms the two prefix products in their original order. Each public PDF/MIS
entry validates its selected immutable context once; the alternatives reuse
that validation without evaluating unused densities or suffixes. Complete-path,
selected-prefix and map-surface validation retain their different contracts.

`PhotonMap::Build` first collects and validates lightweight `(path, vertex)` IDs,
then inserts their radius-relative positions into the hash grid. This temporary
ID vector is released after building; paths/BSDFs are never copied. Querying
sorts and deduplicates candidate IDs before `WithinRadius` and `CompatibleNormals`
apply the physical acceptance rule. The checked broad phase is not the kernel.

The existing `s`, `t`, `light`/`eye` and member naming conventions are retained.
BDPT's classes, rendering loop, MIS, BSDFs and public interfaces are unchanged
by this readability refactor.

## Renderer and example

`integrator\VCM.h` adds the opt-in checked renderer:

```cpp
VertexConnectionMerging vcm(sampler, maxDepth, iterations, initialRadius,
    real(2.0 / 3.0), Vcm::Mode::Combined,
    ThreadedTaskManager::DefaultConcurrency(), false);
bool success = vcm.RenderChecked(scene, camera);
```

`RenderChecked` never saves files; `RenderScene` saves only after success.
No default example, `main()` selection, or CLI behavior changes. The callable
example below and renderer-level acceptance tests cover issue #128; bounded
tests are not a universal convergence or performance claim.

The trailing `useShadingNormals` flag defaults to `false`. Both subpaths call
the current BDPT builders with `SurfaceNormalMode::Geometric`. Setting the flag
to `true` is currently an explicit failure, before path generation or film writes;
it does not silently select geometric normals. Low-level VCM path/map operations
also reject shading-mode records, even when their normals happen to coincide.
BDPT's independent shading-normal option remains supported and unchanged.

The callable example follows the current `GyunityApp\Examples.h/.cpp` interface:

```cpp
bool Examples::VertexConnectionMerging(const std::string& scene = "mesh",
    const Examples::Options& options = {},
    real initialRadius = 0.05f, real alpha = real(2.0 / 3.0),
    Vcm::Mode mode = Vcm::Mode::Combined,
    int concurrency = ThreadedTaskManager::DefaultConcurrency(),
    bool useShadingNormals = false,
    Gyunity::VertexConnectionMerging::Statistics* statistics = nullptr);
```

For example:

```cpp
Examples::Options options;
options.width = options.height = 32;
options.iterations = 16;
options.maxDepth = 6;
if (!Examples::OutputPath("LucyBunnyVcm.hdr", options.output)) return 1;
bool ok = Examples::VertexConnectionMerging("lucy-bunny", options, 0.15f,
    real(2.0 / 3.0), Vcm::Mode::Combined, 1, false);
```

The ten legacy surface scenes and compatible native surface scenes use the shared
`RenderScene` path. `fog`, unknown names, invalid accelerators and unsupported
materials fail explicitly. Defaults remain 128x128, 16 iterations and depth 8;
depth zero is supported. Use `options.iterations`, not `options.spp` or
`options.photonsPerStage`. Dimensions, budgets, missing assets and failed image
writes propagate checked failures, not normal-control-flow exceptions. The
optional statistics pointer is reset on entry and copied after rendering returns,
including failed rendering. A non-null pointer is not needed to render.

Run from the worktree's `GyunityApp` directory: `..\resources` must contain the
registry's assets (`meshs\bunny3.obj`, `meshs\Alucy.obj`, `meshs\water.obj` and
`media\pictures\uffizi-large.hdr` for the representative mesh/env/water scenes).
Defaults use `Test\CornellBoxVcm.hdr` under the **main checkout**, including from
nested worktrees. Explicit output paths are unchanged and their parent directory
must exist. Tests call `Examples::VertexConnectionMerging` directly, without
including `main.cpp`. VCM is not added to the CLI or automatic example batches;
existing PT/BDPT/SPPM/VPT routing and no-argument selection are unchanged.

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

## Explicit path PDF and MIS

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
lost; both masses include the selected lobe's probability in their respective
directions. Main's weighted BSDF sampling, event type and group probability stay
intact; the optional reverse-mass output follows those existing output arguments.
The reverse selection weights are evaluated at the reverse outgoing direction,
not reused from the forward mixture.
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

Current standalone BDPT interfaces, examples and progress behavior are documented
in [BDPT](BDPT.md). Historical VCM acceptance counts and metrics above describe the
pre-main-integration feature snapshot; they do not imply byte-identical images
across subsequent main-branch BSDF, camera or environment corrections.
