# Rendering tests

## Scattering and emissive-surface consistency

GGX full-NDF and visible-normal sampling use their corresponding PDFs; full-NDF
anisotropic samples are unit half-vectors. Reflection PDFs reject opposite-side,
tangent and degenerate direction pairs. `BSDF::Evaluate`, `Sample` and `Pdf`
honor the same lobe flags. The existing Smith masking model is unchanged.

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
