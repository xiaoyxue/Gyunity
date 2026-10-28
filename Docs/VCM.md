# Progressive surface VCM (opt-in)

References: [Georgiev et al., *Light Transport Simulation with Vertex Connection
and Merging*, SIGGRAPH Asia 2012](https://iliyan.com/publications/VertexMerging/VertexMerging_SigAsia2012.pdf)
and [Georgiev, *Implementing Vertex Connection and Merging*, revision 2,
2013](https://iliyan.com/publications/ImplementingVCM/ImplementingVCM_TechRep2012_rev2.pdf).
The structure follows Yuanming Hu's `vcm.cpp` in the legacy Taichi renderer; the
checking style follows Gyunity's own `BDPT.cpp`.

## Reading the implementation

| File | Content |
|---|---|
| `VCM.h/.cpp` | Renderer: boundary checks, the iteration loop, `Connect`, `Merge`, film writes |
| `VCMPath.h/.cpp` | Path densities, strategy enumeration, MIS weight and the primary merge estimate |
| `VCMPhotonMap.h/.cpp` | Hash grid over the light vertices of one iteration |

One iteration reads top-down:

```text
RenderIteration
  radius r_i = r_0 * (i + 1)^((alpha - 1) / 2)
  TraceLightPaths     GenerateLightPath, one attempt per pixel
  BuildPhotonMap      all continuous light surface vertices (VM modes only)
  TraceCameraPaths    per pixel: GenerateCameraPath, then for t = 1..nCamera
    Connect(t)        EvaluateConnection * weightVc for every light prefix s
    Merge(t)          MergeEstimate * weightVm / nVm for every photon near camera[t-1]
  WritePathContribution
```

`Connect` and `Merge` push weighted `Contribution`s into two `PathContribution`s,
as Yuanming's `connect` and `vertex_merge` do. `WritePathContribution` splats the
`t = 1` light-tracing records (already divided by `iterations`) and adds one camera
sample: the VC sum plus the VM sum. Values are `Vector3d`; note that `Vector3d`
scalar operators take `real`, so double factors are passed as `Vector3d(x)`.

BDPT supplies the subpath builders, `EvaluateConnection`, `ConvertDensity` and the
`Bidirectional` probability helpers. BDPT's own renderer and legacy MIS are not used.

## Usage

```cpp
VertexConnectionMerging vcm(sampler, maxDepth, iterations, initialRadius,
    real(2.0 / 3.0), Vcm::Mode::Combined,
    ThreadedTaskManager::DefaultConcurrency(), false);
bool success = vcm.RenderChecked(scene, camera);
```

`RenderChecked` never saves files; `RenderScene` saves only after success. VCM is
reachable through `Examples::VertexConnectionMerging`; it is not in the CLI or the
default example. The radius is in scene units. Retained light paths scale with
resolution times depth, so start small.

Modes: `Combined` uses joint VC/VM MIS; `ConnectionOnly` skips the photon map;
`MergingOnly` keeps VM plus camera-visible `LS*E` emission (every interior event a
pure delta). The three mode images use different MIS denominators and are not
additive components. VC depth is `s+t-2`, VM depth is `s+t-3`; there is no `t=0`
or `(1,1)` strategy.

## Validation

Checks run once, at the boundary, in `Prepare`:

- `useShadingNormals` is rejected (only geometric-normal transport is implemented);
- sampler, depth, iterations, radius, alpha and mode ranges;
- `Bidirectional::ValidateScene` (lights, media, environment), shared with BDPT;
- the sampler's dimension and sample-index budget (below).

After that the renderer trusts its own components: the BDPT path builders, the
photon map's radius and normal filter, and the recorded edge densities. Zero
support is an ordinary outcome (`continue`), not an error. The remaining runtime
failures are genuine: a BDPT helper reporting an invalid density, a sampler that
cannot create a path stream, a BSDF that mixes delta and continuous lobes, a photon
extent the grid cannot index, and a current strategy whose density is zero.

## Sampling

Each iteration traces `P = width * height` light paths and `P` camera paths. Pixel
`k` pairs camera path `k` with light path `k` for VC. Sample indices are
`2 * (iteration * P + pixel) + phase` (light 0, camera 1), independent of threads.
Built-in Halton, Sobol and RegularHalton samplers are pure functions of
(dimension, index) and are shared; other samplers get `CloneForPath(index)` per
path. Paths read samples through BDPT's `RandomStateSequence`.

| Sampler | Dimensions | Maximum depth | Sample indices |
|---|---:|---:|---|
| `HaltonSampler` | 256 | 84 | below `INT64_MAX` |
| `SobolSampler` | 1024 | 340 | below `2^52` |
| `RegularHaltonSampler`, random and custom | 1229 | 409 | below `INT64_MAX` |

A light walk draws `max(5, 3*depth + 2)` samples; a camera walk draws `3*depth + 2`.

## Estimators

- **VC**: `EvaluateConnection(s, t) * weightVc / nVc` with `nVc = 1`.
- **VM**: for photon `y = light[s-1]` within `r` of `x = camera[t-1]`,
  `contributionVm = betaE * f_x(wo, y.wi) * betaL / (pi r^2)`, then
  `weightVm * contributionVm / nVm`. `nVm = P` counts attempts, including empty
  ones. The camera BSDF uses the photon's original incoming direction. There is no
  photon BSDF, extra cosine, visibility ray or division by a full path pdf.
- Products and sums are in double; the camera sample is converted to `real` once.

## MIS

`PathDensities` builds two arrays for the selected physical path: `pdfFwd[i]`, the
density of vertex `i` sampled from the light side, and `pdfPrev[i]`, from the camera
side. Recorded `mPdfFwd`/`mPdfPrev` are reused while their source and sampled
successor remain selected; only the connection or merge boundary is recomputed.
Then (Eq. 10):

```text
VC(s):  pdfPath = prod pdfFwd[0, s)   * prod pdfPrev[s, n)
VM(s):  pdfPath = prod pdfFwd[0, s)   * prod pdfPrev[s-1, n) * pi r^2
weight  = nCurrent * pdfCurrent / (sum_VC pdfPath + nVm * sum_VM pdfPath)
```

A merge samples its vertex from both sides, so its camera prefix starts at `s-1`.
Scores are summed in the log domain against their maximum, so long paths do not
underflow. `UseVc`/`UseVm` select which strategies enter the sum; they never change
the recorded densities.

**Finite-radius merge** ([report, Section 4.3 / Figure 3](https://iliyan.com/publications/ImplementingVCM/ImplementingVCM_TechRep2012_rev2.pdf)).
For light predecessor `a`, photon `y`, camera query `x` and camera predecessor `b`,
both physical endpoints are kept. Sampling `a` backwards uses the BSDF at `x` with
the original directions, converted along `y -> a`; sampling `b` forwards uses the
swapped directions, converted along `x -> b`. No `x-y` edge is introduced.

**Singular convention** (approved "report-keep-bdpt"). An interior delta event
keeps its sampled component mass and geometry (`mDeltaPdf`, `mDeltaReversePdf`),
following report Eqs. (51)-(55). Strategies connecting or merging at a delta vertex
are infeasible. Continuous-path weights equal legacy BDPT's; singular weights may
differ, by design. Point origins are position masses; an environment origin uses
light selection times its directional pdf.

## Photon map

`PhotonMap::Build` stores `(light path index, vertex index)` records for every
continuous surface vertex, at radius-relative coordinates
`(position - firstPhoton) / r`. The grid's broad-phase radius is 2, which
conservatively contains the physical unit disk while float rounding within `2^20`
radii stays below 1/8; a larger extent fails. `Query` returns candidates sorted
and deduplicated in `(path, vertex)` order, filtered by double-precision
`distance^2 <= r^2` and oriented geometric-normal cosine `>= 0.9`. Paths and BSDF
arenas must stay alive until the last query; each concurrent reader owns its result.

## Concurrency, cancellation and output

Each phase is one `ThreadedTaskManager` group over pixels, limited to `concurrency`;
the map build runs in a task arena with the same limit. Thread-local workers own
the light and camera arenas. Camera arenas reset after each pixel; after the camera
phase the map and paths are cleared before the light arenas reset.
`RequestCancellation()` is polled once per pixel; all tasks drain before failure
returns.

`GetStatistics()` reports completed iterations, light attempts, camera samples, the
last radius and the peak photon count. After each successful iteration stderr prints

```text
VCM iteration 64/64: radius 0.00308965, 3804768 photons; elapsed 0:06:54, remaining ~0:00:00
```

## Tests

Run from `GyunityApp` with `GYUNITY_TEST_OUTPUT_DIRECTORY` set to an existing
main-checkout `Test` directory.

- `VCMPath*`, `VCMSingularConvention*`: every strategy weight against two test-only
  oracles, the report's `dVCM/dVC/dVM` recurrence and direct Eq. (10)-(12) products
  (`vcm-weights-release.csv`, `vcm-singular-gate-release.csv`).
- `VCMEstimator*`: the renderer's merge against independent Lambertian, manual
  joint-score and brute-force/report references (`estimator-generated.csv`).
- `VCMPhotonMap*`: queries against an exhaustive oracle.
- `VCMRenderer*`: emission, splat and gather normalization, environment, specular,
  sampler streams, determinism, cancellation and boundary failures.
- `MainExampleRenders.Vertex*`: the example over every surface scene and mode.

Limitations: no media, shading normals, mixed delta/continuous BSDFs, rotated or
multiple environments, adaptive radius or independent photon budget. The
finite-radius kernel, the 0.9 normal cutoff and the `2^20`-radius grid extent are
explicit approximations. Standalone BDPT is documented in [BDPT](BDPT.md).
