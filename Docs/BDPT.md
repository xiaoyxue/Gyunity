# Bidirectional rendering

## Reading the implementation

| File | Responsibility |
|---|---|
| `BDPT.cpp` | Scene validation, tiled rendering, subpath generation, `(s,t)` enumeration and film accumulation |
| `BDPTPath.cpp` | Camera/light origins, surface walks, forward/reverse densities and geometric area conversion |
| `BDPTScattering.h/.cpp` | Normal/frame preparation, shading support, adjoint cosine and checked throughput updates |
| `BDPTConnection.cpp` | Four explicit connection cases, visibility, probability queries and the existing balance MIS |

`EvaluateConnection` returns the primary estimate with its sampling compensation
and visibility, but without MIS. `ConnectPaths` applies `MisWeight` once. The four
cases remain explicit: camera-hit emission (`s=0`), connection to the camera
(`t=1`), connection to a sampled emitter (`s=1`), and two surface endpoints.
Their endpoint PDFs are not interchangeable, so they do not use a universal
vertex-evaluation formula.

The `Bidirectional` connection/PDF helpers are shared with VCM, not duplicated in
its implementation. They retain the current normal-policy and mixed-lobe behavior.
VCM currently requests geometric-normal subpaths and rejects mixed-lobe transport.
The builders also retain successful forward/reverse delta component masses. VCM and
BDPT's optional log-pdf MIS use them; the default ratio MIS keeps its sampled-event flags
and zero markers. See [MIS](#mis).

`Trace` keeps vertex insertion, random draws and forward/reverse density writes
visible. `ScatterThroughput` distinguishes a valid continuation, a null event
and a logged error. Geometric float arithmetic and shading-mode double products
retain their existing order; the refactor does not change the transport model.

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
Media and null-material boundaries are rejected. Mixed delta/continuous BSDFs
record the component actually sampled; their continuous components remain
connectible. Pure mirror/glass events remain supported. A zero-contribution sample stops
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
Media and multiple environment maps are explicitly
unsupported. Environment emission requires finite nonempty scene bounds.

The renderer generates both subpaths, evaluates supported connections at
`depth = s + t - 2`, adds each camera-sample sum once and normalizes camera splats
once by samples per pixel. It does not call the PT direct-light estimator.
The path builders use the integrator's single normal policy.

`MisWeight` (the default, `BdptMis::Ratio`) follows the original PBRT-v3 balance heuristic; the
optional `MisWeightLogPdf` is described in [MIS](#mis). The ratio walk visits neighboring
strategies in both directions using forward/reverse density ratios, then evaluates
`1 / (1 + sumRatios)`. Scalar accumulation uses double precision. Only temporary
connection-adjacent densities are changed in separate light/camera MIS snapshots.
These snapshots replace PBRT's `ScopedAssignment` restoration mechanism: the
input paths are const and never modified, including on early returns. Endpoint
and predecessor references and the two ratio walks follow PBRT's ordering. Ordinary
zero-density events are distinct from delta-event markers. Point-light camera-hit
strategies, specular connections, `(1,1)` and `t=0` are excluded.

Mixed materials remain connectible through their continuous lobes, but an
alternative split adjacent to an **internally sampled delta event** cannot
reproduce that path and is excluded from the MIS denominator in both walks.
This tests the sampled-event flags, not just material capabilities. The actual
connection endpoints' unused delta selections are still cleared in the local
snapshots; density-ratio advancement and light-endpoint admissibility are unchanged.
`BDPTConnectionTest.cpp` checks internal camera/light mixed events against
independent strategy products, all-continuous controls, and a reflected emitter
whose only supported strategy is camera-hit emission. Its small Plastic highlight
render also checks PT and BDPT against independent dielectric Fresnel radiance.

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

## MIS

BDPT offers two evaluations of the balance heuristic over the `(s,t)` strategies of a
path: `BdptMis::Ratio` (default, `MisWeight`) and `BdptMis::LogPdf` (`MisWeightLogPdf`,
`--bdpt-mis logpdf` in GyunityApp). They agree on every path without an interior delta
event. Equation numbers below appear in the code comments as `Eq. (n)`.

Write the selected path in light-to-camera order $x_0 \ldots x_{n-1}$, $n = s + t$.
Each vertex has two area densities (pdfPos), sampled from the light side and from the
camera side:

$$\text{pdfFwd}[i] = p_\omega(x_{i-1} \to x_i \mid x_{i-2})\, G(x_{i-1} \to x_i), \qquad
\text{pdfPrev}[i] = p_\omega(x_{i+1} \to x_i \mid x_{i+2})\, G(x_{i+1} \to x_i) \tag{1}$$

with $G(u \to v) = |\cos\theta_v| / \lVert v - u\rVert^2$ on the geometric normal
(`DirectionToArea`). `pdfFwd[0]` is the light-selection PMF times the origin's pdfPos
(pdfDir for an environment), and an environment source contributes its launch-disk
pdfPos times $|\cos\theta_v|$. Recorded `mPdfFwd`/`mPdfPrev` are reused while their source
and that source's predecessor are selected; only the four densities at the connection
are recomputed with `PdfPos`/`LightOriginPdf`. Strategy $k$ (light prefix $x_0 \ldots
x_{k-1}$, camera prefix $x_k \ldots x_{n-1}$) has the path density

$$p(k) = \prod_{i=0}^{k-1}\text{pdfFwd}[i] \prod_{i=k}^{n-1}\text{pdfPrev}[i] \tag{2}$$

and the weight of the sampled strategy $s$, evaluated in the log domain with
$\ell_k = \log p(k)$ and $m = \max_k \ell_k$, is

$$w_s = \frac{p(s)}{\sum_k p(k)} = \frac{e^{\ell_s - m}}{\sum_k e^{\ell_k - m}} \tag{3}$$

A strategy is excluded when it connects at a vertex that scattered a delta event of the
path, when `ValidConnection` rejects it, or when a factor is zero. Prefix and suffix sums
of the logs give all $\ell_k$ in $O(n)$.

**Delta vertices.** At an interior vertex $x_k$ that sampled a delta component, the
singular distribution is removed but its sampled component mass $m$ (`mDeltaPdfFwd`
forward, `mDeltaPdfPrev` reverse, including lobe selection) and the edge geometry are kept:

$$\text{pdfFwd}[k+1] = m_{fwd}(x_k)\, G(x_k \to x_{k+1}), \qquad
\text{pdfPrev}[k-1] = m_{rev}(x_k)\, G(x_k \to x_{k-1}) \tag{4}$$

This is VCM's convention (Docs/VCM.md Eq. (11), report Eqs. 51-55), so `MisWeightLogPdf`
equals `Vcm::ComputeMisWeight` in `ConnectionOnly` mode on the same subpaths.

**Why the two evaluations differ.** Both densities of Eq. (4) contain a Dirac
distribution. It is the same reflection or refraction constraint seen from either side,
so it cancels in the ratio of any two feasible strategies; what remains is
$m\,G$. The ratio walk instead replaces both densities by 1. That drops the factor
$m_{fwd}\, G(x_k \to x_{k+1}) / (m_{rev}\, G(x_k \to x_{k-1}))$ from the ratio between
strategies on the two sides of $x_k$. The result is still unbiased, because every
strategy of a path is weighted with the same modified densities and the weights still
sum to one. It is no longer the balance heuristic, though, so its variance is not that of
the balance heuristic. For example, on the L-D-S-D-E fixture in `VCMPathTest.cpp` the two
evaluations differ in the strategy ratio by exactly $\text{separation}^2$. Both cost O(n)
per weighted connection; `LogPdf` also allocates two density vectors and evaluates n logs.

Tests:
- `BDPTConnectionTest.cpp` checks `MisWeightLogPdf` against hand products for continuous,
  mirror and mixed-lobe chains.
- The generated mirror, glass, environment and point-light paths in `VCMPathTest.cpp`
  require it to equal VCM `ConnectionOnly` on every positive connection.
- The Plastic highlight render requires both evaluations to match the analytic radiance.

## Smooth shading normals
With `useShadingNormals = true` (`--bdpt-normals shading` in GyunityApp), BDPT follows
the #148 contract instead of replacing interpolated normals:

- `ns` (interpolated, normalized) defines the BSDF frame and scattering cosines. The
  tangent comes from `dpdu` projected onto the shading plane, independent of the ray;
  a degenerate parameterization uses a deterministic `CoordinateSystem` basis.
  `ng` keeps area densities (`DirectionToArea`), sidedness, emission and ray offsets.
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
density separation, one-time correction on both subpaths and emitter sidedness.
The independent display-operator checks live in `ToneMapTest.cpp`.
