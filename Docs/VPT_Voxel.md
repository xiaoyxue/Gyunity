# VPT + Voxel Density: Minimal Pseudocode

## Assumptions

- One bounded volume with a nonnegative scalar density grid and trilinear interpolation.
- Constant scalar `sigma_s`, `sigma_a`; isotropic phase function `1 / (4 * pi)`.
- Normalized world-space rays; the volume box is not a shading surface.
- Surface BSDFs and light sampling come from an existing path tracer.

```text
sigma_t = sigma_s + sigma_a
majorant = max(grid) * sigma_t
density(p) = trilinear(grid, world_to_grid(p))   // Zero outside the volume.
```

## 1. Sample a Medium Collision: Delta Tracking

```text
SampleCollision(ray, t_end):
    [a, b] = intersect(volume_box, ray) clipped to [0, t_end]
    if interval is empty or majorant == 0: return NONE
    t = a
    loop:
        t += Exponential(rate = majorant)
        if t >= b: return NONE
        if Uniform01() < density(ray(t)) * sigma_t / majorant:
            return t
        // Rejected candidate: null collision; keep walking.
```

## 2. Shadow Transmittance: Ratio Tracking

```text
Transmittance(ray, t_end):
    [a, b] = intersect(volume_box, ray) clipped to [0, t_end]
    if interval is empty or majorant == 0: return 1
    T = 1
    t = a
    loop:
        t += Exponential(rate = majorant)
        if t >= b: return T
        T *= 1 - density(ray(t)) * sigma_t / majorant

DirectLight(x, scattering_kernel):
    (wi, distance, Li, pdf) = SampleLight(x)   // PDF includes light selection.
    shadow = OffsetRay(x, wi)
    if pdf == 0 or Occluded(shadow, distance): return 0
    return scattering_kernel(wi) * Li * Transmittance(shadow, distance) / pdf
```

## 3. Volumetric Path Tracing

```text
Trace(ray):
    L = 0; beta = 1; previous_delta = true
    for depth in [0, max_depth):
        (hit, t_surface) = IntersectSurface(ray)   // Infinity on a miss.
        t = SampleCollision(ray, t_surface)

        if t != NONE:
            x = ray(t)
            beta *= sigma_s / sigma_t            // Implicit absorption.
            if beta == 0: break
            L += beta * DirectLight(x, kernel = 1 / (4 * pi))
            wi = UniformSphere()                // Phase / PDF = 1.
            previous_delta = false
        else:
            if previous_delta:
                L += beta * EmissionOrEnvironment(hit, ray)
            if hit is NONE: break
            x = hit.position
            L += beta * DirectLight(x, kernel = BSDF(hit, -ray.dir, wi) * abs_cos(wi))
            (wi, f, pdf, is_delta) = SampleBSDF(hit, -ray.dir)
            if pdf == 0: break
            beta *= f * abs_cos(wi) / pdf
            previous_delta = is_delta

        ray = OffsetRay(x, wi)
        if depth >= roulette_start:
            q = clamp(max_component(beta), 0.05, 0.95)
            if Uniform01() >= q: break
            beta /= q
    return L
```

**Important:** Do not multiply continuation throughput by transmittance again:
`SampleCollision` already samples survival through the medium. Explicit
transmittance is needed for shadow connections. This minimal version uses
light-sampled next-event estimation without MIS: count directly hit emission
only for camera/specular paths to avoid double counting, and include every
emitter (including the environment) in `SampleLight`.

The legacy Taichi `voxel` implementation uses this style of delta tracking;
`sdf_voxel` additionally skips empty regions. Its shadow estimator uses binary
collision survival rather than the ratio-tracking estimator shown here.
