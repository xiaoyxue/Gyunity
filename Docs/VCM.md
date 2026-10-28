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

One iteration reads top-down. Equation numbers refer to the formulas below and
appear in the code comments as `Eq. (n)`.

```text
RenderIteration
  radius r_i                                                      Eq. (1)
  TraceLightPaths     GenerateLightPath, one attempt per pixel
  BuildPhotonMap      all continuous light surface vertices (VM modes only)
  TraceCameraPaths    per pixel: GenerateCameraPath, then for t = 1..nCamera
    Connect(t)        EvaluateConnection * weightVc / nVc         Eq. (2)
    Merge(t)          MergeEstimate * weightVm / nVm              Eq. (3), (4)
  WritePathContribution                                           Eq. (5)
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

Iteration `i` (from 0) uses one radius for the map, MIS and normalization:

$$r_i = r_0\,(i+1)^{(\alpha-1)/2} \tag{1}$$

A light subpath has `s` vertices $y_0 \ldots y_{s-1}$ (light origin first); a camera
subpath has `t` vertices $z_0 \ldots z_{t-1}$ (camera first). `mThroughput` is the
sampling-compensated throughput $\beta$ of each prefix.

**Vertex connection** of light prefix `s` and camera prefix `t`, with
$f_{VC}$ from BDPT's `EvaluateConnection` (throughputs, both BSDFs, geometry term
and visibility) and $n_{VC} = 1$:

$$C_{VC}(s,t) = \frac{w_{VC}(s,t)}{n_{VC}}\, f_{VC}(s,t) \tag{2}$$

**Vertex merging** of photon $y = y_{s-1}$ with camera vertex $x = z_{t-1}$ when
Eq. (12) accepts them. The camera BSDF is evaluated with the photon's original
incoming direction $\omega_y$ (`y.mOutDir`); there is no photon BSDF, extra cosine,
visibility ray or division by a full path pdf:

$$C^{prim}_{VM}(s,t) = \frac{\beta_x\, f_x(\omega_x, \omega_y)\, \beta_y}{\pi r_i^2} \tag{3}$$

$$C_{VM}(s,t) = \frac{w_{VM}(s,t)}{n_{VM}}\, C^{prim}_{VM}(s,t), \qquad n_{VM} = P \tag{4}$$

$n_{VM}$ counts light-path **attempts**, including empty ones; it is neither the
photon count nor the number of query hits.

**Film.** Each camera path adds one sample; light tracing ($t = 1$) splats onto its
projected pixel, already divided by the iteration count $N$:

$$I_{pixel} \mathrel{+}= \sum_{t \ge 2,\,s} C_{VC}(s,t) + \sum_{t,\,y} C_{VM}(s,t), \qquad
I_{splat} \mathrel{+}= \frac{C_{VC}(s,1)}{N} \tag{5}$$

Products and sums are in double; each film value is converted to `real` once.

## MIS

Write the full path in light-to-camera order $x_0 \ldots x_{n-1}$, with
$n = s + t$ for VC and $n = s + t - 1$ for VM (the merge vertex counts once).
`PathDensities` builds two area densities per vertex: `pdfFwd[i]` samples $x_i$
from the light side, `pdfPrev[i]` from the camera side,

$$\text{pdfFwd}[i] = p_\omega(x_{i-1} \to x_i \mid x_{i-2})\,
\frac{|\cos\theta_{x_i}|}{\lVert x_i - x_{i-1}\rVert^2}, \qquad
\text{pdfPrev}[i] = p_\omega(x_{i+1} \to x_i \mid x_{i+2})\,
\frac{|\cos\theta_{x_i}|}{\lVert x_i - x_{i+1}\rVert^2} \tag{6}$$

where $\theta$ uses the geometric normal (`ConvertDensity`). `pdfFwd[0]` is the light
selection probability times the origin's area density (directional density for an
environment). Recorded `mPdfFwd`/`mPdfPrev` are reused while their source and
sampled successor remain selected; only the connection or merge boundary is
recomputed. The path densities of strategy `s` are (report Eq. 10):

$$p_{VC}(s) = \prod_{i=0}^{s-1}\text{pdfFwd}[i] \prod_{i=s}^{n-1}\text{pdfPrev}[i] \tag{7}$$

$$p_{VM}(s) = \pi r_i^2 \prod_{i=0}^{s-1}\text{pdfFwd}[i] \prod_{i=s-1}^{n-1}\text{pdfPrev}[i] \tag{8}$$

A merge samples its vertex $x_{s-1}$ from both sides, so its camera product starts
at $s-1$. The balance heuristic over all enabled VC strategies ($s = 0 \ldots n-1$)
and VM strategies ($s = 2 \ldots n-1$) is evaluated in the log domain,
with $\ell_k = \log(n_k\, p_k)$ and $m = \max_k \ell_k$:

$$w_{cur} = \frac{n_{cur}\, p_{cur}}{\sum_{VC} n_{VC}\, p_{VC}(s) + \sum_{VM} n_{VM}\, p_{VM}(s)}
= \frac{e^{\ell_{cur} - m}}{\sum_k e^{\ell_k - m}} \tag{9}$$

so long paths do not underflow. `UseVc`/`UseVm` choose which strategies enter the
sum; they never change the densities. A strategy is infeasible (excluded) when it
connects or merges at a delta vertex or has a zero factor.

**Finite-radius merge** ([report, Section 4.3 / Figure 3](https://iliyan.com/publications/ImplementingVCM/ImplementingVCM_TechRep2012_rev2.pdf)).
For light predecessor $a = y_{s-2}$, photon $y$, camera query $x$ and camera
predecessor $b = z_{t-2}$, both physical endpoints are kept and the BSDF at $x$
scatters with the original directions; no $x$–$y$ edge is introduced:

$$\text{pdfFwd}[s] = p_x(\omega_y \to \omega_x)\, G(x \to b), \qquad
\text{pdfPrev}[s-2] = p_x(\omega_x \to \omega_y)\, G(y \to a) \tag{10}$$

with $G(u \to v) = |\cos\theta_v| / \lVert v - u\rVert^2$.

**Singular convention** (approved "report-keep-bdpt", report Eqs. 51-55). At an
interior delta vertex $x_k$ the singular distribution is removed, but its sampled
component mass $m$ (`mDeltaPdf` forward, `mDeltaReversePdf` reverse) and geometry
are kept:

$$\text{pdfFwd}[k+1] = m_{fwd}(x_k)\, G(x_k \to x_{k+1}), \qquad
\text{pdfPrev}[k-1] = m_{rev}(x_k)\, G(x_k \to x_{k-1}) \tag{11}$$

Continuous-path weights equal legacy BDPT's; singular weights may differ, by design.
Point origins are position masses and cannot be hit by a camera path.

## Photon map

`PhotonMap::Build` stores `(light path index, vertex index)` records for every
continuous surface vertex, at radius-relative coordinates
`(position - firstPhoton) / r`. The grid's broad-phase radius is 2, which
conservatively contains the physical unit disk while float rounding within `2^20`
radii stays below 1/8; a larger extent fails. `Query` returns candidates sorted
and deduplicated in `(path, vertex)` order, then accepts a photon $y$ at camera
vertex $x$ in double precision when

$$\lVert x - y\rVert^2 \le r_i^2, \qquad \hat n_x \cdot \hat n_y \ge 0.9 \tag{12}$$

with oriented geometric normals. Paths and BSDF arenas must stay alive until the
last query; each concurrent reader owns its result.

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
