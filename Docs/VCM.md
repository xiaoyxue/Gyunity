# Progressive surface VCM (opt-in)

References: [Georgiev et al., *Light Transport Simulation with Vertex Connection
and Merging*, SIGGRAPH Asia 2012](https://iliyan.com/publications/VertexMerging/VertexMerging_SigAsia2012.pdf)
and [Georgiev, *Implementing Vertex Connection and Merging*, revision 2,
2013](https://iliyan.com/publications/ImplementingVCM/ImplementingVCM_TechRep2012_rev2.pdf).
Each iteration runs in stages: trace light paths, build the photon map, then trace
camera paths. The checking style follows Gyunity's own `BDPT.cpp`.

## Reading the implementation

| File | Content |
|---|---|
| `VCM.h/.cpp` | Renderer: boundary checks, the iteration loop, `Connect`, `Merge`, film writes |
| `VCMPath.h/.cpp` | Path densities, strategy enumeration, MIS weight and the primary merge estimate |
| `VCMPhotonMap.h/.cpp` | Hash grid over the light vertices of one iteration |
| `VCMPhotonGrid.h/.cpp` | Compact parallel photon grid of the recursive integrator |
| `VCMRecursive.h/.cpp` | `RecursiveVertexConnectionMerging`: the same renderer with recursive MIS and `PhotonGrid` |
| `VCMRecursiveMis.h/.cpp` | The report's dVCM/dVC/dVM recurrence, Eqs. (14)-(16) |

One iteration reads top-down. Equation numbers refer to the formulas below and
appear in the code comments as `Eq. (n)`.

```text
RenderIteration
  radius r_i                                                      Eq. (1)
  TraceLightPaths     GenerateLightPath, nVm attempts
  BuildPhotonMap      all continuous light surface vertices (VM modes only)
  TraceCameraPaths    nVm samples: GenerateCameraSample, then for t = 1..nCamera
    Connect(t)        EvaluateConnection * weightVc / nVc         Eq. (2)
    Merge(t)          MergeEstimate * weightVm / nVm              Eq. (3), (4)
  WritePathContribution                                           Eq. (5)
```

`Connect` and `Merge` push weighted `Contribution`s into two `PathContribution`s.
`WritePathContribution` splats the
`t = 1` light-tracing records (already scaled by `SplatScale`) and adds one camera
sample: the VC sum plus the VM sum. Values are `Vector3d`; note that `Vector3d`
scalar operators take `real`, so double factors are passed as `Vector3d(x)`.

BDPT supplies the subpath builders, `EvaluateConnection`, `DirectionToArea` and the
`Bidirectional` probability helpers. BDPT's own renderer and legacy MIS are not used.

## Usage

```cpp
VertexConnectionMerging vcm(sampler, maxDepth, iterations, initialRadius,
    real(2.0 / 3.0), Vcm::Mode::Combined,
    ThreadedTaskManager::DefaultConcurrency(), useShadingNormals);
bool success = vcm.RenderChecked(scene, camera);
```

`RecursiveVertexConnectionMerging` takes the same arguments and produces the same image;
only its MIS weights are evaluated differently (see [Recursive MIS](#recursive-mis)). The
examples select it with `Options::recursiveMis`, and `--example veach-bidir-vcm-recursive`
renders the SmallVCM comparison with it.

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

With `P = width * height` and stage frequency `f` (taichi's `stage_frequency`,
default 1), each iteration traces `nVm = ceil(P / f)` light paths and `nVm` camera
paths, so one film pass of a camera path per pixel is split over `f` iterations.
Camera sample `k` pairs with light path `k` for VC, so $n_{VC} = 1$ at any `f`, and
iteration `i` places it in pixel `(k + i * nVm) mod P` with a uniform jitter. The `f`
iterations of a pass therefore reach every pixel; rendering requires at least `f`
iterations. With `f = 1` this is one camera path per pixel, as in the paper and
SmallVCM. Sample indices are `2 * (iteration * nVm + k) + phase` (light 0, camera 1),
independent of threads.

`f` exists to bound per-iteration memory at large resolutions; it is not free.
Merges see only the iteration's `nVm` light paths, and the same spp needs `f` times
the iterations, so the radius of Eq. (1) shrinks further.
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

$$C_{VM}(s,t) = \frac{w_{VM}(s,t)}{n_{VM}}\, C^{prim}_{VM}(s,t), \qquad n_{VM} = \lceil P / f \rceil \tag{4}$$

$n_{VM}$ counts light-path **attempts**, including empty ones; it is neither the
photon count nor the number of query hits.

**Film.** Each camera path adds one filtered sample at its film position, which the
film averages by the samples a pixel actually received; light tracing ($t = 1$) splats
onto its projected pixel. The camera's importance is normalized over the whole film,
so the $n_{VM}$ light paths of each of the $N$ iterations are read as a per-pixel
average through $P / (n_{VM} N)$:

$$I_{pixel} \mathrel{+}= \sum_{t \ge 2,\,s} C_{VC}(s,t) + \sum_{t,\,y} C_{VM}(s,t), \qquad
I_{splat} \mathrel{+}= \frac{P}{n_{VM}\, N}\, C_{VC}(s,1) \tag{5}$$

**Light-tracing MIS count.** Per pixel, the camera-side strategies are drawn
$n_j$ times with $P$ times the film-normalized density, and light tracing $n_{VM}$
times. Relative to the camera side its count is $n_{VM} / (n_j P)$, which is 1 in
expectation because camera and light share the count: $E[n_j] = n_{VM} / P$. Eq. (9)
therefore keeps $n_{VC} = 1$ for every connection, $t = 1$ included. Decoupling the
two counts would make that factor $n_{light} / n_{camera}$.

Products and sums are in double; each film value is converted to `real` once.

**Shading normals** (`useShadingNormals = true`) use BDPT's adjoint-corrected
transport on both subpaths: light-path throughputs carry the importance factor
$K_{imp}$, camera-path throughputs the radiance factor $K_{rad}$, and VC is BDPT's
shading-mode `EvaluateConnection`. Densities and MIS still use geometric normals;
shading normals only mask BSDF support, also at the merge boundary of Eq. (10).
Merging needs one correction. Photons land with area density
$p_\omega\,\lvert\omega_y \cdot n_g(y)\rvert / d^2$, so Eq. (3) implicitly carries
$y$'s geometric cosine, while the VC estimator of the same path carries
$K_{rad}(x) = \lvert\omega_y \cdot n_s(x)\rvert$ (zero outside the shading
support). In shading mode Eq. (3) is therefore scaled by

$$\frac{K_{rad}(x;\, \omega_x, \omega_y)}{\lvert \omega_y \cdot n_g(y) \rvert} \tag{13}$$

which is 1 when $n_s = n_g$; geometric mode skips it, so its output is unchanged.

## MIS

Write the full path in light-to-camera order $x_0 \ldots x_{n-1}$, with
$n = s + t$ for VC and $n = s + t - 1$ for VM (the merge vertex counts once).
`PathDensities` builds two area densities per vertex: `pdfFwd[i]` samples $x_i$
from the light side, `pdfPrev[i]` from the camera side,

$$\text{pdfFwd}[i] = p_\omega(x_{i-1} \to x_i \mid x_{i-2})\,
\frac{|\cos\theta_{x_i}|}{\lVert x_i - x_{i-1}\rVert^2}, \qquad
\text{pdfPrev}[i] = p_\omega(x_{i+1} \to x_i \mid x_{i+2})\,
\frac{|\cos\theta_{x_i}|}{\lVert x_i - x_{i+1}\rVert^2} \tag{6}$$

where $\theta$ uses the geometric normal (`DirectionToArea`). `pdfFwd[0]` is the light
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
component mass $m$ (`mDeltaPdfFwd` forward, `mDeltaPdfPrev` reverse) and geometry
are kept:

$$\text{pdfFwd}[k+1] = m_{fwd}(x_k)\, G(x_k \to x_{k+1}), \qquad
\text{pdfPrev}[k-1] = m_{rev}(x_k)\, G(x_k \to x_{k-1}) \tag{11}$$

Continuous-path weights equal legacy BDPT's; singular weights may differ, by design.
Point origins are position masses and cannot be hit by a camera path.

## Recursive MIS

`VertexConnectionMerging` evaluates Eq. (9) directly: every connection and every photon
rebuilds both density arrays of the whole path and sums all strategies, O(n^2) with heap
allocations. `RecursiveVertexConnectionMerging` returns the same weights through the report's
recurrence ([report, Section 5](https://iliyan.com/publications/ImplementingVCM/ImplementingVCM_TechRep2012_rev2.pdf);
SmallVCM's `vertexcm.hxx`), so each subpath is summarized once and each weight costs O(1).

Both subpaths are summarized the same way, each in its own sampling order. For subpath vertex
$i$, `along`$_i$ is its Eq. (6) density from its subpath predecessor (`pdfFwd` on a light
subpath, `pdfPrev` on a camera subpath) and `against`$_i$ its density from its successor, both
with the Eq. (11) delta masses. $c_i \in \{0, 1\}$ says the connection joining vertices $i-1$
and $i$ is feasible and enabled (for $i = 0$: a light subpath's non-delta emitter, hit by the
camera; a camera subpath has none), $m_i \in \{0, 1\}$ says a merge at $i$ is ($i \ge 1$, continuous),
and $\eta = n_{VM}\,\pi r_i^2$, or 0 without merging:

$$dVCM_i = \frac{c_i}{\text{along}_i}, \quad
dVC_i = \frac{\eta\, m_{i-1} + dVCM_{i-1} + \text{against}_{i-2}\, dVC_{i-1}}{\text{along}_i}, \quad
dVM_i = \frac{m_{i-1} + dVCM_{i-1}/\eta + \text{against}_{i-2}\, dVM_{i-1}}{\text{along}_i} \tag{14}$$

with $dVC_0 = dVM_0 = 0$. $dVCM_i$ scores the connection ending at vertex $i$, and $dVC_i$
every strategy further back, both relative to a connection at $i$; $dVM_i = dVC_i / \eta$ scores
the same strategies relative to a merge at $i$.

A **connection** $(s, t)$ of light end $y$ and camera end $z$ recomputes only its boundary
densities (Eq. 6, as in `ComputeMisWeight`):

$$w_{VC} = \frac{1}{1 + w_L + w_C}, \quad
w_L = \text{pdfPrev}[s-1]\,\bigl(\eta\, m_y + dVCM_y + \text{pdfPrev}[s-2]\, dVC_y\bigr), \quad
w_C = \text{pdfFwd}[s]\,\bigl(\eta\, m_z + dVCM_z + \text{pdfFwd}[s+1]\, dVC_z\bigr) \tag{15}$$

Light tracing ($t = 1$) has $w_C = 0$ and an emitter hit ($s = 0$) has $w_L = 0$. For $s = 0$ the
camera end $z$ is the light origin, so $m_z = 0$ and $dVCM_z$ uses connection $(1, t-1)$, which
needs no continuous lobe at $z$.

A **merge** of photon $y = y_{s-1}$ at camera vertex $x = z_{t-1}$ needs only the two Eq. (10)
densities:

$$w_{VM} = \frac{1}{1 + dVCM_y/\eta + \text{pdfPrev}[s-2]\, dVM_y + dVCM_x/\eta + \text{pdfFwd}[s]\, dVM_x} \tag{16}$$

These are SmallVCM's updates (`dVC = cos/pdf * (dVC * revPdf + dVCM + eta)`, and `dVM`
likewise) and weights (`ConnectVertices`, the merge query). SmallVCM works in solid angle and
folds each cosine and squared distance into the next update; Gyunity's builders already record
area densities, so Eqs. (10) and (11) carry over unchanged. SmallVCM also samples a fresh light
for $s = 1$; here $s = 1$ connects to the light path's own origin and is one more connection.

The summary is an O(n) pass after each subpath is traced. A connection then evaluates two to
four `PdfPos` densities and a merge two BSDF densities; nothing is allocated.

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

## Photon grid

`RecursiveVertexConnectionMerging` stores its photons in `PhotonGrid`, after SmallVCM's
`hashgrid.hxx`; `VertexConnectionMerging` keeps `PhotonMap`. Both accept exactly the photons of
Eq. (12); only the order of a query's results differs.

- **Cells.** A photon at $p$ lies in cell $\lfloor p / 2r \rfloor$ per axis and is stored once.
  A query at $x$ visits the cells from $\lfloor (x - r') / 2r \rfloor$ to
  $\lfloor (x + r') / 2r \rfloor$, with $r' = r\,(1 + 10^{-6})$: usually 2x2x2, and each distinct
  bucket once. The floor is monotone, so every photon within $r$ is in a visited cell.
- **Records.** Each record holds the position, geometric normal, path and vertex index (32 bytes),
  so Eq. (12) reads only the record, never the light path.
- **Build.** Photons are counted and placed per path in parallel, keyed by
  `bucket << 32 | index`, sorted with `tbb::parallel_sort`, and indexed by per-bucket ranges in a
  power-of-two table. Keys are unique, so the order is deterministic and queries need no sort.
  `Clear` keeps the storage for the next iteration.
- Cell coordinates must stay below $2^{62}$; there is no `2^20`-radius extent limit.

On veach-bidir (1024^2, f = 1, depth 7, 128 threads) one iteration takes 10.5 s with
`VertexConnectionMerging`, 8.3 s with recursive MIS and `PhotonMap`, and 2.0 s with `PhotonGrid`.
Before the grid, photon queries were about 80% of the camera phase, a query sorted and then
dereferenced about 20 times more candidates than it accepted, and the build ran on one core.

## Concurrency, cancellation and output

Each phase is one `ThreadedTaskManager` group over the iteration's paths, limited to `concurrency`;
the map build runs in a task arena with the same limit. Thread-local workers own
the light and camera arenas. Camera arenas reset before each camera sample; after the camera
phase the map and paths are cleared before the light arenas reset.
`RequestCancellation()` is polled once per path; all tasks drain before failure
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
- `VCMPhotonGrid*`: accepted sets equal `PhotonMap` and the exhaustive oracle at random,
  radius-boundary and cell-wall queries; deterministic order, rebuilds, failures.
- `VCMRecursive*`: every connection and merge weight of generated paths (closed and open
  box, mirror and glass walls, area, point and environment origins, both normal modes, all
  three modes) against `ComputeMisWeight`, and renders of both integrators.
- `VCMRenderer*`: emission, splat and gather normalization, environment, specular,
  sampler streams, determinism, cancellation and boundary failures.
- `MainExampleRenders.Vertex*`: the example over every surface scene and mode.

Limitations: no media, shading normals, mixed delta/continuous BSDFs, rotated or
multiple environments, adaptive radius or independent photon budget. The
finite-radius kernel, the 0.9 normal cutoff and the `2^20`-radius grid extent are
explicit approximations. Standalone BDPT is documented in [BDPT](BDPT.md).
