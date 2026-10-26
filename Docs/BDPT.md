# Bidirectional rendering

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
Both builders take an explicit `SurfaceNormalMode`, which must be the same for the
two subpaths of a render. `Geometric` builds shading frames from geometric normals
(the initial BDPT model). `Shading` keeps interpolated normals; see
[Smooth shading normals](#smooth-shading-normals).
Media, null-material boundaries and mixed delta/continuous BSDFs are rejected.
Pure mirror/glass events remain supported. A zero-contribution sample stops
extension without losing the surface vertex already reached.
If conversion of a non-delta edge to area density yields zero, the walk stops
before appending that vertex and retains its valid prefix. This covers rounded
nearby intersections without an epsilon cutoff or remapping a continuous zero
to one. Delta-generated zero-density markers still propagate through the path.

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
`BidirectionalPathTracing(sampler, maxDepth, spp, useShadingNormals = false)` supports
finite area/point lights and one environment light, alone or mixed with finite lights.
The default selects the geometric-normal model; `true` selects corrected smooth
shading normals for both subpaths.
`RenderChecked(scene, camera)` returns success/failure explicitly; the shared
`RenderScene` checks this result before saving the film. Other integrators retain
their existing behavior through the default checked-render adapter.
Media, mixed delta/continuous BSDFs and multiple environment maps are explicitly
unsupported. Environment emission requires finite nonempty scene bounds.

The renderer generates both subpaths, evaluates supported connections at
`depth = s + t - 2`, adds each camera-sample sum once and normalizes camera splats
once by samples per pixel. It does not call the PT direct-light estimator.
The path builders use the integrator's single normal policy.

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

`BDPTAcceptanceTest.cpp` directly exercises all four supported strategy families,
isolates camera splats with primary rays that miss all geometry, checks splat
normalization at multiple sample counts, expands auxiliary environment launch
bounds without changing geometry, and checks latitude/longitude signs using a
directional HDR marker map. Generated fixtures and images use
`GYUNITY_TEST_OUTPUT_DIRECTORY` under the main checkout's `Test` directory.

`Examples::BidirectionalPathTracing` accepts a registered scene name and
`Examples::Options`, including output, dimensions, spp, depth and accelerator.
It supports the original surface builders and native scenes, rejects `fog`,
and does not change the default no-argument PT example. A C++ call is:

```cpp
Examples::Options options;
if (!Examples::OutputPath("CornellBoxBdptLucyBunny.png", options.output)) return 1;
if (!Examples::BidirectionalPathTracing("lucy-bunny", options)) return 1;
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

## Smooth shading normals
With `useShadingNormals = true` (`--bdpt-normals shading` in GyunityApp), BDPT follows
the #148 contract instead of replacing interpolated normals:

- `ns` (interpolated, normalized) defines the BSDF frame and scattering cosines. The
  tangent comes from `dpdu` projected onto the shading plane, independent of the ray;
  a degenerate parameterization uses a deterministic `CoordinateSystem` basis.
  `ng` keeps area densities (`ConvertDensity`), sidedness, emission and ray offsets.
  A shading normal on the far side of `ng` is a checked failure.
- `Bidirectional::ScatteringCosine` returns `K = |wi.ns| * C`, with
  `C = |wo.ns| |wi.ng| / (|wo.ng| |wi.ns|)` for importance and `1` for radiance
  (Veach / PBRT-v3 `CorrectShadingNormal`). It is applied exactly once to extension,
  `t = 1`, `s = 1` and interior connections; connections then keep only distance
  falloff and an area emitter's geometric cosine. Densities and event masses are never
  multiplied by `K`.
- Support: `wo` and `wi` must each lie strictly on the same side of `ns` and `ng`.
  Outside it, extension stops without retrying, connections contribute zero and
  queried directional PDFs are zero; accepted samples keep their original densities.
- Area lights orient emission, emitted-direction sampling and `PdfLight` by `ng`.
  Triangle and sphere samples return unit normals with `ng` oriented like ray
  intersections, and area-to-solid-angle conversions use `ng`.

Shading-normal transport is an approximate model: strongly tilted normals can still
show shadow-terminator artifacts or high variance near grazing angles; they are not
clamped. `BDPTShadingNormalTest.cpp` covers the analytic factors, support, frames,
density separation, one-time correction on both subpaths, emitter sidedness and the
PBRT tone-mapping operators.
