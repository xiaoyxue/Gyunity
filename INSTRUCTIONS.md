# Command-line usage

Open a Visual Studio developer terminal in the Gyunity checkout/worktree root.
Paths below are repository-relative. Generated renders, logs and temporary
imports belong in the **main checkout's** `Test` directory, even from a worktree.
Runtime assets under the worktree's `resources` remain separate.

## PowerShell setup and build

```powershell
$TestRoot = Join-Path (Resolve-Path (Join-Path (git --no-pager rev-parse --path-format=relative --git-common-dir) '..')).Path 'Test'
New-Item -ItemType Directory -Force $TestRoot | Out-Null
git submodule update --init --recursive
.\init.ps1
msbuild .\Gyunity.sln /m:2 /p:Configuration=Release /p:Platform=x64 /v:minimal
```

Use `Configuration=Debug` for a debug build. `init.ps1` prepares both dependency
configurations; it recreates `external\build`, so it is not needed before every
incremental solution build. CMake must be on PATH; VS2026 needs CMake 4.2 or newer.
The Git-relative expression computes `$TestRoot` without a machine-specific path
and keeps it valid when the current directory changes.

## Render from PowerShell

After the setup above:

```powershell
Push-Location GyunityApp
..\x64\Release\GyunityApp.exe --help
..\x64\Release\GyunityApp.exe --list-scenes
..\x64\Release\GyunityApp.exe --list-examples
..\x64\Release\GyunityApp.exe --scene CornellBoxStandard --integrator path --width 256 --height 256 --spp 16 --max-depth 8 --out "$TestRoot\cornell-path.png"
..\x64\Release\GyunityApp.exe --scene veach-mis --integrator bdpt --width 512 --height 288 --spp 64 --out "$TestRoot\mis-bdpt.png"
..\x64\Release\GyunityApp.exe --scene veach-bidir --width 256 --height 256 --spp 32 --out "$TestRoot\bidir.png"
..\x64\Release\GyunityApp.exe --scene CornellBoxStandard --integrator sppm --width 256 --height 256 --iterations 16 --photons-per-stage 32768 --max-depth 8 --out "$TestRoot\cornell-sppm.png"
Pop-Location
```

`veach-bidir` defaults to BDPT; `--integrator path` explicitly selects PT.
SPPM has independent `--iterations` and `--photons-per-stage` budgets;
`--spp` is rejected with SPPM. Its defaults are 256 iterations and 300000
photons per stage. PT/BDPT use `--spp`. Native JSON defaults apply unless an
explicit flag overrides them. `--accel bvh|kd` selects the accelerator.
Output supports `.png`, `.bmp`, `.hdr` and Float32 `.exr`.

PT and VPT count **scattering events** in `maxDepth`, including medium phase
scattering. Null medium boundaries do not consume depth. Emission directly
visible to the camera or reached after the last delta scattering is credited
before stopping; no further scattering or next-event estimate is taken at the
limit. The C++ integrators support depth zero for directly visible emission
(attenuated through media in VPT); this does not change CLI budget validation.
Continuous BSDF/phase continuations do not add emission a second time: their
direct illumination estimator already accounts for it.

VPT's C++ `equalAngular` option uses the existing equi-angular estimator only
when every scene light is a concrete `AreaLight` or `PointLight`. With any
environment or other light implementation (including subclasses), the **whole
scene** uses free-flight sampling and the standard direct-illumination estimator,
exactly as with `equalAngular=false`. Choosing this fallback separately for a
selected light would incorrectly mix a conditional light estimate with an
all-lights estimate. Point PDFs are initialized and checked before division.
The existing supported-light equi-angular segment-length limitation (#165 L6)
is unchanged; this fallback is not a general correction of that estimator.

For already-generated native JSON, use
`--scene-file ..\resources\scenes\native-demo\scene.json` instead of `--scene`.
The renderer does not execute Python.

`Run` in `main.cpp` handles entry-point selection. `--example NAME` uses the
example's documented defaults and a default output filename under `Test`:

```powershell
Push-Location GyunityApp
..\x64\Release\GyunityApp.exe --example light-tracing
..\x64\Release\GyunityApp.exe --example matpreview-copper
..\x64\Release\GyunityApp.exe --example feature-scenes
..\x64\Release\GyunityApp.exe --example all
Pop-Location
```

Prepare Matpreview first. `feature-scenes` runs original Cornell/MIS/bidir and
all nine Matpreview profiles through PT, BDPT and SPPM. `all` also runs the five
legacy integrator examples. These batches can take considerable time and are
never run implicitly. No arguments still selects the original PT/environment
demo. Use `--scene` flags or the C++ `Examples::Options` API for smaller budgets.

## Windows Command Prompt (cmd.exe)

Use Developer Command Prompt, starting in the checkout/worktree root.
These are interactive commands; double `%` loop variables when putting them
in a `.cmd` file.

```bat
for /f "delims=" %G in ('git --no-pager rev-parse "--path-format=relative" --git-common-dir') do for %T in ("%G\..\Test") do set "GYUNITY_TEST=%~fT"
if not exist "%GYUNITY_TEST%" mkdir "%GYUNITY_TEST%"
git submodule update --init --recursive
call init.cmd
msbuild Gyunity.sln /m:2 /p:Configuration=Release /p:Platform=x64 /v:minimal
cd GyunityApp
..\x64\Release\GyunityApp.exe --list-scenes
..\x64\Release\GyunityApp.exe --scene veach-bidir --width 256 --height 256 --spp 32 --out "%GYUNITY_TEST%\bidir.png"
..\x64\Release\GyunityApp.exe --scene matpreview-copper --integrator sppm --iterations 16 --photons-per-stage 32768 --out "%GYUNITY_TEST%\copper-sppm.png"
cd ..
```

Do not run both dependency setup scripts for the same build. `init.cmd` and
`init.ps1` are alternatives. Python/uv commands below work in either shell;
replace PowerShell's `$TestRoot` with cmd's `%GYUNITY_TEST%` when needed.

## Temporary gallery asset checkout

The published image repository can be cloned beneath the main checkout's
ignored `.worktree` directory. From the setup above:

```powershell
$GyunityRoot = Split-Path $TestRoot
git clone https://github.com/xiaoyxue/GyunityAssets "$GyunityRoot\.worktree\gyunity-assets"
git -C "$GyunityRoot\.worktree\gyunity-assets" worktree add ..\gyunity-assets-gallery -b user/your-alias/scene-gallery origin/main
```

Choose your own branch alias and reuse an existing clone rather than cloning
over it. Make publication changes in the asset worktree's
`GyunityAssets\Gallery`, not its main checkout. Render PNGs and keep logs under
`$TestRoot\gallery-publication` first, then copy only completed images into the
Gallery. Do not upload raw Matpreview geometry, source environments or supplied
reference images. The asset repository stores gallery outputs; it does not
replace the original ZIP inputs needed by the scene importers.

## Python authoring and import commands (uv)

Return to the checkout/worktree root and use the PowerShell `$TestRoot` defined
above. Place the original user-supplied ZIPs in `$TestRoot\archives` before
running archive imports; they are local inputs, not committed assets.
Install uv on PATH before running these commands.

```powershell
uv sync --project tools\gyscene --locked
uv run --project tools\gyscene --locked gyscene build tools\gyscene\examples\native_demo.py --out "$TestRoot\custom-scene.json"
uv run --project tools\gyscene --locked gyscene validate "$TestRoot\custom-scene.json"
uv run --project tools\gyscene --locked gyscene schema --out tools\gyscene\scene.schema.json
uv run --project tools\gyscene --locked gyscene import-mitsuba --archive-dir "$TestRoot\archives" --scene cornell-box --output-root resources\scenes\mitsuba
uv run --project tools\gyscene --locked gyscene import-mitsuba-file resources\scenes\mitsuba\cornell-box\scene.xml --name CornellBoxStandard --output-root "$TestRoot\xml-imports"
uv run --project tools\gyscene --locked --extra pbrt gyscene import-pbrt tools\gyscene\examples\pbrt_demo.pbrt --rgb-radiance-scale 1 --output-root "$TestRoot\pbrt-imports"
uv run --project tools\gyscene --locked --extra mitsuba gyscene import-matpreview "$TestRoot\archives\matpreview.zip" --profile all --max-depth 8 --output-root resources\scenes
```

Use `veach-mis` or `veach-bidir` for the other reviewed archive imports.
Matpreview's batch import decodes the original geometry once and prepares
`matpreview-diffuse`, `matpreview-glass`, `matpreview-copper`,
`matpreview-checkerboard`, `matpreview-mirror`, `matpreview-rough-glass`,
`matpreview-copper-beckmann`, `matpreview-copper-anisotropic` and
`matpreview-plastic` for direct `--scene` use.

The original Matpreview archive has no license file, so its raw/derived assets
remain local and ignored by Git. All exterior materials are explicit native
overrides; native plastic now uses the source's smooth layered model. Old
roughness-based Plastic JSON must be regenerated in a fresh output location.
The PBRT
adapter supports a bounded static RGB subset and requires an explicit RGB
radiance scale, not a claim of spectral equivalence.

The same operations have runnable Python examples:

```powershell
uv run --project tools\gyscene --locked python tools\gyscene\examples\native_demo.py --out "$TestRoot\python-scene.json"
uv run --project tools\gyscene --locked python tools\gyscene\examples\import_mitsuba.py --archive-dir "$TestRoot\archives" --scene veach-bidir --output-root "$TestRoot\python-imports"
uv run --project tools\gyscene --locked --extra pbrt python tools\gyscene\examples\import_pbrt.py tools\gyscene\examples\pbrt_demo.pbrt --rgb-radiance-scale 1 --output-root "$TestRoot\python-imports"
uv run --project tools\gyscene --locked --extra mitsuba python tools\gyscene\examples\import_matpreview.py "$TestRoot\archives\matpreview.zip" --profile all --max-depth 8 --output-root resources\scenes
```

In a native scene script, SPPM settings are explicit and separate from `spp`:

```python
from gyscene import RenderSettings, SppmSettings

scene.render = RenderSettings(
    integrator="sppm",
    width=256,
    height=256,
    max_depth=8,
    sppm=SppmSettings(iterations=16, photons_per_stage=32768),
)
```

## Test and regression scripts

Tests require their dependencies and a built app. Use an existing directory
under `$TestRoot`; agent-driven test commands run through test subagents.

```powershell
New-Item -ItemType Directory -Force "$TestRoot\checks" | Out-Null
$env:GYUNITY_TEST_OUTPUT_DIRECTORY = "$TestRoot\checks"
$env:GYUNITY_MITSUBA_ARCHIVES = "$TestRoot\archives"
$env:GYUNITY_MATPREVIEW_ARCHIVE = "$TestRoot\archives\matpreview.zip"
$env:GYUNITY_MATPREVIEW_SCENES = (Resolve-Path resources\scenes\matpreview).Path
uv run --project tools\gyscene --locked --all-extras python -m unittest discover -s tools\gyscene\tests -v
Push-Location GyunityApp
..\x64\Release\GyunityTest.exe --gtest_filter="SceneRegistry.*:SceneCommandLine.*:MainExamples.*:MainExampleRenders.*"
Pop-Location
```

For previously generated matching reference images, choose either regression
runner; both invoke the real application and fail on missing references or
render/hash errors:

```powershell
.\GyunityTest\RunSceneRegression.ps1 -OutputDirectory "$TestRoot\regression" -ReferenceDirectory "$TestRoot\references" -Scenes mesh,slope,sphere,env,triangle,textured,empty,diffuse,water,fog
uv run --no-project .\GyunityTest\RunSceneRegression.py --output-directory "$TestRoot\python-regression" --reference-directory "$TestRoot\references" --scenes mesh slope sphere env triangle textured empty diffuse water fog
```

See [Docs/Examples.md](Docs/Examples.md), [Docs/SceneAuthoring.md](Docs/SceneAuthoring.md),
[Docs/Testing.md](Docs/Testing.md) and [tools/gyscene/README.md](tools/gyscene/README.md)
for API semantics, supported input subsets and validation details.
