# Rendering tests

## Light tracing transport

LT depth D permits D scattering vertices after the light origin; D=0 connects
only that origin without accessing a surface slot. A zero-density or zero-value
sample terminates the walk while retaining its finite prefix. Mixed BSDFs
retain typed, weighted sampling and classify the sampled event, not the whole
material. No shading-normal/adjoint correction is introduced here.

The stored origin throughput remains Le. Its camera splat is restricted to
area lights emitting toward the camera and applies camera importance/PDF,
geometric endpoint cosine, light-selection PMF and position density exactly
once, independent of the sampled emission-direction density. Point and
environment origins do not splat directly. LT tests the floating raster against
half-open film bounds before splatting, without integer truncation.

`LTTransportAudit.*`, `LightTracingEmission.*` and `MixedBsdfIntegrators.Lt*`
cover these contracts. Depth expectations use analytic mirror throughput and
vertex counts, not PT's depth convention.

`RenderChecked` returns false, with nonthrowing error logging, for invalid
settings, malformed densities or nonfinite transport contributions. Path
generation returns -1 on failure; `ConnectToCameraChecked` exposes the camera
connection status. Legitimate zero-density/value, occluded, back-facing and
off-film samples remain successful null events. The legacy `Render` and
`ConnectToCamera` wrappers retain their signatures and log failures; callers
that need status must use the checked methods.

## Scattering and emissive-surface consistency

GGX full-NDF and visible-normal sampling use their corresponding PDFs; full-NDF
anisotropic samples are unit half-vectors. Reflection PDFs reject opposite-side,
tangent and degenerate direction pairs. `BSDF::Evaluate`, `Sample` and `Pdf`
honor the same lobe flags. The existing Smith masking model is unchanged.

Microfacet normal samples use the canonical positive-z hemisphere for either
view side, consistently with native `D` and `Pdf`. Rough transmission evaluation
and PDFs require both directions to agree with the reconstructed microfacet and
macrosurface orientation; opposite macrohemispheres alone are insufficient.
Tangent and zero-half-vector continuous queries return zero rather than NaN.
Plastic uses explicit exterior-to-coating Fresnel IORs; rough glass retains its
existing exterior-to-interior IOR order. These contracts have actual-material
and component regressions in `TestBsdfSample.*`.

The native GGX model retains separable Smith masking, as in Mitsuba, rather than
PBRT-v3's height-correlated masking. The PBRT-style artist-roughness mapping is
also unchanged: roughness `0.1` maps to alpha approximately `0.462`, not raw alpha
`0.1`. These model/parameter differences must be matched explicitly in reference
comparisons. Native Plastic now uses Mitsuba-style smooth layering, not GGX.

For mixed delta/continuous BSDFs, `IsDelta` means all lobes are delta; `Sample`
reports the event actually selected. Weighted lobe selection and filtered PDFs
use the same probabilities, while delta masses are never added to continuous
densities. Direct-light estimation excludes delta components. PT/VPT emission
and BDPT MIS use the sampled event, and SPPM compensates the probability of
recording a continuous visible point. An iteration with no visible points skips
photon tracing rather than querying an empty hash grid. Plastic and mixed-lobe
regressions cover these contracts, including geometric-normal filtering.

PT, VPT and the SPPM eye pass accumulate emitted radiance and reflected direct
lighting independently, including after a specular bounce and on an emitter's
nonemitting back side. VPT estimates direct surface lighting only at non-delta
BSDFs. SPPM records emission before choosing a delta, continued-glossy or
visible-point branch; its first-hit photon exclusion remains unchanged.

These contracts are covered by `TestMicrofacetDistribution.*`,
`TestBsdfSample.*`, `PathTracingEmission.*`,
`VptAndSppm/EmissiveSurfaceIntegrators.*` and `SppmSurfaceEmission.*` in
`GyunityTest`. SPPM fixtures use positive iteration/photon budgets and the
public renderer; set `GYUNITY_TEST_OUTPUT_DIRECTORY` under the main checkout's
`Test` for their HDR outputs.

## Scene regression

Run the registered-scene regression without patching or recompiling `main.cpp`:

```powershell
# From the worktree root, after building Release|x64:
$TestRoot = Join-Path (Resolve-Path (Join-Path (git --no-pager rev-parse --path-format=relative --git-common-dir) '..')).Path 'Test'
.\GyunityTest\RunSceneRegression.ps1 `
  -OutputDirectory "$TestRoot\scene_registry_step4\renders" `
  -ReferenceDirectory "$TestRoot\remaining_scenes_step3"
```

Use `-Scenes mesh,slope,sphere,env,triangle,textured,empty,diffuse,water,fog` when
comparing against the historical ten-scene references; `lucy-bunny` is new and
has no original-scene baseline. Without `-Scenes`, the runner requires references
for every registered scene rather than silently skipping missing references.

A Python equivalent is available without additional packages:

```powershell
uv run --no-project .\GyunityTest\RunSceneRegression.py `
  --output-directory "$TestRoot\python-scene-regression" `
  --reference-directory "$TestRoot\remaining_scenes_step3" `
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

## Scattering and emissive-surface consistency

GGX full-NDF and visible-normal sampling use their corresponding PDFs; full-NDF
anisotropic samples are unit half-vectors. Reflection PDFs reject opposite-side,
tangent and degenerate direction pairs. `BSDF::Evaluate`, `Sample` and `Pdf`
honor the same lobe flags. The existing Smith masking model is unchanged.

PT accumulates emitted radiance and reflected direct lighting independently:
an emitting diffuse surface still reflects illumination, including after a
specular bounce and on its nonemitting back side. These contracts are covered
by `TestMicrofacetDistribution.*`, `TestBsdfSample.*` and
`PathTracingEmission.*` in `GyunityTest`.
