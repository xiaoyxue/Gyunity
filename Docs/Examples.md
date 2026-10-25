# C++ integrator examples

`GyunityApp\Examples.h/.cpp` exposes the existing five integrator examples and
all original-data feature scenes as public, checked C++ functions. `Run` lives
in `main.cpp`, where it handles example selection and calls `RunSceneCommandLine`
for ordinary scene commands. `main()` retains the outermost exception boundary.
Examples contain no command-line parsing and do not depend on SceneCommandLine;
both use the shared `SceneRenderer` configuration/loading/rendering API.
With no arguments, the application still runs the original PT/environment demo,
not the entire suite.

| Function | Builder scene | Default settings | Output |
|---|---|---|---|
| `Examples::PathTracing` | `env` | 1024x1024, 100 spp, depth 10 | `Test\CornellBoxPtEnvmap.bmp` |
| `Examples::Sppm` | `mesh` | 1024x1024, 256 iterations, 300000 photons/stage, depth 20, radius 0.05, alpha 0.66666667 | `Test\CornellBoxSppmTest.bmp` |
| `Examples::LightTracing` | `mesh` | 1024x1024, 512 spp, depth 15 | `Test\CornellBoxLT.bmp` |
| `Examples::VolumePathTracing` | `fog` | 1024x1024, 2000 spp, depth 8 | `Test\MediaTestFogEquiAngular.bmp` |
| `Examples::BidirectionalPathTracing` | Any non-volume scene; default `mesh` | 1024x1024, 256 spp, depth 8 | `Test\CornellBoxBdpt.bmp` |

`Examples::CornellBoxStandard`, `Examples::VeachMis`, `Examples::VeachBidir`
and `Examples::Matpreview(profile)` load native JSON through the same defaults
and validation as the scene CLI. They preserve its camera, filter, seed,
resolution, spp, depth and accelerator unless overridden, and accept
`Examples::SurfaceIntegrator::Path`, `Bdpt` or `Sppm`. `VeachBidir` defaults to BDPT;
the other explicit native helpers default to PT. Named examples honor their
native JSON integrator default, including BDPT for `veach-bidir`. Default filenames are
`Test\<scene-name>-path.png`, `Test\<scene-name>-bdpt.png` or `Test\<scene-name>-sppm.png`.

From `GyunityApp`, no recompilation is needed to choose an example:

```powershell
..\x64\Release\GyunityApp.exe --list-examples
..\x64\Release\GyunityApp.exe --example light-tracing
..\x64\Release\GyunityApp.exe --example veach-mis
..\x64\Release\GyunityApp.exe --example matpreview-copper
..\x64\Release\GyunityApp.exe --example feature-scenes
..\x64\Release\GyunityApp.exe --example all
```

`feature-scenes` runs CornellBoxStandard, Veach MIS/bidir and all nine
Matpreview profiles with **PT, BDPT and SPPM**. `all` also runs the five legacy
integrator examples. Batches are explicitly opt-in, check that the registered
JSON files exist before rendering, use separate filenames and stop with an
error on failure; missing Matpreview data is never silently skipped.
These default workloads can take considerable time. `--example` accepts a
name only; use `--scene NAME --integrator path|bdpt|sppm --out FILE [options]` to
customize a native scene from the CLI, or `Examples::Options` for C++/batch settings.

Default outputs always go to `Test` at the **main repository root**, not the
working directory or a worktree's root. The examples locate the main checkout
from the executable's ancestors, skipping nested worktrees' `.git` files, and
create `Test` if necessary. Keep worktrees under the main checkout's `.worktree`
directory as described in `AGENTS.md`. Default filenames have no numeric suffix.

The legacy builder examples default to BVH, a box filter and their registered
scene camera. PathTracing, LightTracing and VolumePathTracing retain `RandomSampler(123)`; SPPM retains
Halton sampling, per-hit-point radius shrinking and glossy-ray tracing.
VolumePathTracing retains its volume-specific settings and the fog builder's
long-lived medium. Existing builders use the registry's look-at camera;
native JSON scenes preserve their camera-to-world basis and vertical FOV.

Scene construction, asset checks and checked image saving are shared with the
CLI through `SceneRenderer.h/.cpp` (`LoadSceneDefaults` and `RenderScene`).
Only the CLI module owns parsed command state; renderer options have no CLI fields.
Run from `GyunityApp` for
asset paths. `Examples::Options` has optional width, height, spp, maxDepth and
accelerator overrides, plus SPPM iterations/photonsPerStage. `output` overrides
one filename; `outputDirectory` selects an existing directory for default
filenames, including batch output. They are mutually exclusive, and batches
reject a single filename. Omitted values retain each example's defaults.

```cpp
Examples::Options options;
options.width = 341;
options.height = 256;
options.spp = 64;
options.maxDepth = 8;
options.iterations = 16;
options.photonsPerStage = 32768;
if (!Examples::VeachMis(options, Examples::SurfaceIntegrator::Bdpt)) return 1;
if (!Examples::Matpreview("rough-glass", options, Examples::SurfaceIntegrator::Path)) return 1;
// Explicitly run every feature scene using the same overrides.
if (!Examples::FeatureScenes(options)) return 1;
```

The settings above intentionally preserve the representative legacy demos and
are separate from the CLI's 512x512 / 64 spp / depth-8 defaults. Missing resources
or failed image writes report an error and return a nonzero application exit.

The example smoke tests use small workloads through the same functions and
check readable, nonblack HDR outputs. Set an existing output directory under the
main checkout's `Test` to enable them; otherwise these render tests are skipped:

```powershell
# From the worktree's GyunityApp directory:
$TestRoot = Join-Path (Resolve-Path (Join-Path (git --no-pager rev-parse --path-format=relative --git-common-dir) '..')).Path 'Test'
New-Item -ItemType Directory -Force "$TestRoot\scene_builder_main" | Out-Null
$env:GYUNITY_TEST_OUTPUT_DIRECTORY = "$TestRoot\scene_builder_main"
..\x64\Debug\GyunityTest.exe --gtest_filter="MainExamples.*:MainExampleRenders.*"
..\x64\Release\GyunityTest.exe --gtest_filter="MainExamples.*:MainExampleRenders.*"
```
