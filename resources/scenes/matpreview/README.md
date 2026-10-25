# Original material-preview import (local assets)

The Mitsuba gallery `matpreview.zip` is a different asset from LTE Orb. It
contains no license file, so this repository does **not** redistribute its
raw/derived geometry, EXR environment or supplied reference renders. Use the
user-provided local archive; do not infer CC0 from the other gallery packages.

Prepare all named examples once, from the worktree root:

```powershell
$TestRoot = Join-Path (Resolve-Path (Join-Path (git --no-pager rev-parse --path-format=relative --git-common-dir) '..')).Path 'Test'
uv run --project tools\gyscene --locked --extra mitsuba gyscene import-matpreview "$TestRoot\archives\matpreview.zip" --profile all --max-depth 8 --output-root resources\scenes
```

Place the original ZIP in the main checkout's `Test\archives` first.
See [command instructions](../../../INSTRUCTIONS.md) for PowerShell and cmd.exe setup.
This decodes the source once and creates nine JSON files sharing the same
local meshes/environment. Generated files are git-ignored. A single profile
name still imports just that example.

| Profile / `--scene` suffix | Explicit native exterior |
|---|---|
| `diffuse` | Source green RGB (0.240, 0.771, 0.361), one-sided diffuse |
| `glass` | Smooth dielectric, source interior IOR 1.9 |
| `copper` | GGX conductor, raw alpha (0.1, 0.1) |
| `checkerboard` | Diffuse checkerboard using original mesh UVs, scale (8, 8) |
| `mirror` | Ideal specular mirror, reflection (1, 1, 1) |
| `rough-glass` | Native GGX dielectric, roughness (0.1, 0.1), interior IOR 1.9 |
| `copper-beckmann` | Copper with Beckmann distribution, raw alpha (0.1, 0.1) |
| `copper-anisotropic` | GGX copper, raw alpha (0.05, 0.25) |
| `plastic` | Native diffuse-plus-GGX plastic, source green diffuse color, roughness 0.1 |

These cover all six existing surface material families. The three copper
variants share eta (0.200438, 0.924033, 1.10221) and k
(3.91295, 2.45285, 2.14219). Rough glass/plastic use native artist-roughness
mapping, not raw alpha. Native plastic retains its own fixed Fresnel parameters;
it is **not** the original Mitsuba plastic model. Every exterior change remains
an explicit override in metadata.

A custom Python script can pass any supported native material to
`import_preview`, or a material dictionary to `import_previews` for a shared
batch. See `tools\gyscene\README.md`.

All profiles share the original three meshes, world-space normals/UVs,
checkerboard floor, camera and EXR lighting. Mitsuba's serialized-mesh decoder
is used only during import, not during Gyunity rendering. Original EXR bytes
are read natively with OpenEXR; direction conventions are converted with a
matrix, without resampling or converting the environment to RGBE.
The C++ path uses checked OpenEXRCore status APIs, not exception handling.

The source requests unlimited depth, so a finite `--max-depth` override is
required and recorded. Gaussian filtering and supported one-sided diffuse
materials are retained. Current BDPT geometric-normal shading and camera
splat-filter limitations still apply.

From `GyunityApp`, call any registered example directly (no Python runtime
needed after preparation):

```powershell
Set-Location GyunityApp
..\x64\Release\GyunityApp.exe --list-scenes
..\x64\Release\GyunityApp.exe --scene matpreview-copper --integrator path --spp 64 --out "$TestRoot\matpreview-copper.png"
..\x64\Release\GyunityApp.exe --scene matpreview-rough-glass --integrator bdpt --spp 64 --out "$TestRoot\matpreview-rough-glass.exr"
..\x64\Release\GyunityApp.exe --scene matpreview-copper --integrator sppm --iterations 16 --photons-per-stage 32768 --out "$TestRoot\matpreview-copper-sppm.png"
..\x64\Release\GyunityApp.exe --example matpreview-plastic
```

Replace the suffix with any profile in the table. `--example NAME` uses its
defaults and writes `Test\<name>-path.png`; `--scene` supports explicit render
overrides and requires `--out`. Missing local assets report the preparation
command rather than selecting another scene.

`Examples::Matpreview(profile, options, integrator)` is the public C++ entry.
`--example feature-scenes` explicitly renders all original Cornell/MIS/bidir
and Matpreview entries using PT, BDPT and SPPM; `--example all` also runs the five
legacy integrator examples. Neither batch runs by default.

To keep an imported package elsewhere, choose another `--output-root` and
render its JSON with `--scene-file`; named registry entries use the local
`resources\scenes\matpreview` package.

Choose `.png` for a display preview. Generated images must be labeled as
Gyunity results and not confused with the archive's supplied images.
