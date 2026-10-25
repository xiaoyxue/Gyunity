# Native scene authoring

`gyscene` is Gyunity's Python scene model, not a representation of Mitsuba's
plugin tree. Native scripts and import adapters use the same typed models,
validation, JSON serializer and generated schema. The renderer needs neither
Python nor uv once JSON and referenced assets exist.

From the worktree root:

```powershell
$TestRoot = Join-Path (Resolve-Path (Join-Path (git --no-pager rev-parse --path-format=relative --git-common-dir) '..')).Path 'Test'
uv sync --project tools\gyscene --locked
uv run --project tools\gyscene --locked gyscene build tools\gyscene\examples\native_demo.py --out resources\scenes\native-demo\scene.json
uv run --project tools\gyscene --locked gyscene import-mitsuba --archive-dir "$TestRoot\archives" --scene cornell-box --output-root resources\scenes\mitsuba
uv run --project tools\gyscene --locked gyscene schema --out tools\gyscene\scene.schema.json
uv run --project tools\gyscene --locked gyscene validate resources\scenes\mitsuba\cornell-box\scene.json
```

Place original user-supplied ZIPs in the main checkout's `Test\archives`
before importing. `$TestRoot` is resolved from Git rather than a machine-specific
directory, and remains valid after changing the working directory.
See [INSTRUCTIONS.md](../../INSTRUCTIONS.md) for cmd.exe and PowerShell usage.

Repeat import with `veach-mis` and `veach-bidir`. The imported Cornell scene is
named **CornellBoxStandard**. Import checks pinned archive SHA-256 values and
retains the original CC0 license, XML and all referenced OBJ files unchanged.
The reviewed `veach-bidir` preset defaults to BDPT, recording its native
integrator override separately from the source XML's path integrator. Generic
standalone XML imports retain the source integrator convention.
Supplied reference images are not copied into production assets and must not
be presented as renderer output.

## Python entry points

All entry points construct the same `gyscene.Scene` and serialize our native
JSON. Python code is in the repository, not hidden in a renderer executable:

| Purpose | Runnable example | Reusable module |
|---|---|---|
| Author a custom scene in Python | `examples\native_demo.py` | `gyscene.model` |
| Parse original Mitsuba XML/OBJ packages | `examples\import_mitsuba.py` | `gyscene.mitsuba` |
| Parse the supported PBRT v4 surface subset | `examples\import_pbrt.py` | `gyscene.pbrt` |
| Import original material-preview geometry with an explicit supported BSDF | `examples\import_matpreview.py` | `gyscene.matpreview` |

```powershell
uv run --project tools\gyscene --locked python tools\gyscene\examples\native_demo.py --out "$TestRoot\custom-scene.json"
uv run --project tools\gyscene --locked python tools\gyscene\examples\import_mitsuba.py --archive-dir "$TestRoot\archives" --scene cornell-box --output-root "$TestRoot\python-imports"
uv run --project tools\gyscene --locked python tools\gyscene\examples\import_mitsuba.py resources\scenes\mitsuba\cornell-box\scene.xml --name CornellBoxStandard --output-root "$TestRoot\xml-imports"
uv run --project tools\gyscene --locked --extra pbrt python tools\gyscene\examples\import_pbrt.py tools\gyscene\examples\pbrt_demo.pbrt --rgb-radiance-scale 1 --output-root "$TestRoot\python-imports"
```

The equivalent CLI is `gyscene import-pbrt FILE --output-root DIRECTORY
--rgb-radiance-scale SCALE`. PLY import uses the optional `pbrt` dependency
group; inline triangle meshes and spheres do not need it.

Standalone Mitsuba XML uses `gyscene import-mitsuba-file FILE --output-root
DIRECTORY [--name NAME] [--max-depth N]`, or `gyscene.mitsuba.import_xml`.
The bounded adapter supports source matrix/translate/scale/rotate/look-at
transforms and FOV axes, and copies referenced OBJ/license files unchanged.
Unlimited source depth requires an explicit finite native depth budget.
Unimplemented plugins still fail rather than being silently approximated.

### PBRT v4 subset and explicit differences

The parser follows [PBRT v4's file format](https://www.pbrt.org/fileformat-v4):
typed scalar/array parameters, comments, Include (including gzip), named
materials, attribute scopes, coordinate systems and static transforms.
Includes and referenced meshes resolve from the **initial input directory**;
paths escaping that source package are rejected. PBRT file matrices are
column-major and camera transforms are world-to-camera; both are converted
explicitly to the native conventions. PBRT FOV applies to the smaller axis.

Supported geometry is full spheres under positive uniform transforms,
inline triangle meshes and triangular PLY meshes, retaining positions,
normals, UVs, winding and original source files. Supported materials are
diffuse, smooth dielectric and rough RGB-parameter conductors. PBRT v4's
roughness remapping is **sqrt(roughness)**, not the v3 polynomial.

Specify `Camera "perspective"`, `Sampler "independent"`, `Integrator "path"`
or `"bdpt"`, and an explicit `PixelFilter`. The current filter subset is box
radius 0.5 or triangle radius 1. Missing declarations would otherwise select
PBRT defaults that the native backend cannot faithfully reproduce, so they
are rejected. RGB Film defaults are 1280x720. Arbitrary shaders, textures,
volumes, instances, animation, spectral/blackbody parameters, clipped shapes
and unsupported filter settings fail explicitly; this is not a full PBRT parser.

**Light units are not silently copied.** PBRT v4 photometrically normalizes
its light spectra. `--rgb-radiance-scale` is therefore required: it explicitly
chooses the native RGB scale after normalization by linear-sRGB luminance.
Scale 1 means unit native RGB luminance before the source light's `scale`
factor. This is a documented RGB projection, not PBRT spectral or film
calibration equivalence. Source files and the conversion choice remain in
metadata. Unsupported spectra are rejected, not guessed.

`pbrt_demo.pbrt` is an authored API example, **not** a replacement for any
gallery dataset. Generated OBJ files are data-format conversions of the
actual source triangles, not guessed replacement geometry.

## Local material-preview profiles

The original `matpreview.zip` uses serialized geometry, an EXR environment,
checkerboard reflectance and Gaussian filtering. Its plastic exterior is
not silently mapped to another material. Select an explicit supported profile:

```powershell
uv run --project tools\gyscene --locked --extra mitsuba python tools\gyscene\examples\import_matpreview.py "$TestRoot\archives\matpreview.zip" --profile diffuse --max-depth 8 --output-root "$TestRoot\material-preview"
```

Use `--profile all` to decode the original meshes once and generate all nine
registered examples: `diffuse`, `glass`, `copper`, `checkerboard`, `mirror`,
`rough-glass`, `copper-beckmann`, `copper-anisotropic` and `plastic`.
For direct `--scene matpreview-NAME` invocation, use `--output-root resources\scenes`
from the worktree root; that local Matpreview package is git-ignored.

A custom script may pass its own native material to `import_preview`, or a
name-to-material dictionary to `import_previews` for a single-decode custom batch.
Set `$env:GYUNITY_TEST_OUTPUT_DIRECTORY = $TestRoot` in PowerShell before running
the script:

```python
from pathlib import Path
import os
from gyscene import Mirror, Plastic, RoughDielectric
from gyscene.matpreview import import_previews

test_root = Path(os.environ["GYUNITY_TEST_OUTPUT_DIRECTORY"])
import_previews(
    test_root / "archives" / "matpreview.zip",
    test_root / "custom-previews",
    profiles={
        "custom-mirror": Mirror(reflection=(0.9, 0.9, 0.9)),
        "custom-plastic": Plastic(diffuse=(0.2, 0.7, 0.3), roughness=0.2),
        "custom-rough-glass": RoughDielectric(roughness=(0.1, 0.3), eta_inside=1.5),
    },
    max_depth=8,
)
```

The native plastic profile uses Gyunity's existing diffuse-plus-GGX material
with its fixed IOR, **not** the original Mitsuba plastic BSDF. Adding native
material types does not expand the supported Mitsuba/PBRT adapter vocabulary.
The real geometry/camera/UVs/normals and
EXR bytes remain unchanged apart from converting serialized mesh arrays to
world-space OBJ. The optional Mitsuba package only decodes source data; it is
not used for Gyunity rendering.

The archive has no license file, so raw/derived preview assets remain local
and are not redistributed by this repository. Supplied reference renders are
not copied as generated output. See `resources\scenes\matpreview\README.md`
for rendering commands, provenance and limitations.

## Native version 1

JSON uses `schema_version: 1`, `name`, `render`, `camera`, named `materials`,
ordered `shapes`, optional environment `lights`, and non-rendering `metadata`. Unknown fields, invalid numbers,
unresolved material references, duplicate IDs and missing mesh files fail.
Author scripts export a `Scene` named `scene`; `build` executes trusted Python.

Transforms are affine row-major 4x4 matrices multiplying column vectors.
Camera columns are **right, up, forward, position** and `fov_y` is in degrees.
The Mitsuba adapter explicitly converts its left/up/forward camera basis and
horizontal FOV; it does not invent another camera. Mesh/quad/box transforms
are object-to-world. Quads store four ordered vertices; boxes store explicit
lower/upper bounds; spheres currently use world-space center/radius only.
Relative mesh paths are resolved from the JSON directory, with either path
separator accepted by Python.

Colors/radiance are linear RGB. Conductors store raw `alpha` (X, Y), complex
IOR and explicit `ggx` or `beckmann` distribution: no roughness remapping.
Diffuse materials can be one- or two-sided and accept constant RGB or a native
`Checkerboard` reflectance. Conductors currently require two-sided behavior;
smooth dielectric IORs and reflection/transmission are explicit.
`Mirror` exposes ideal specular reflection. `RoughDielectric` exposes the
existing GGX model with exterior IOR fixed at 1; `Plastic` exposes the existing
diffuse-plus-GGX model with its fixed 1.5/1 Fresnel parameters. Their
`roughness` values lie in (0,1] and use the renderer's existing
`GGXDistribution::RoughnessToAlpha` mapping, unlike conductors' raw `alpha`.
Rough dielectric has two roughness axes, plastic one. Neither accepts a
distribution or unsupported IOR override. All six material types share the
same Python/schema/C++ validation path. Area emission is
attached to a shape independently of its material. One `Environment` light
may reference a linear RGB HDR/EXR map, positive scale and orthonormal
direction-only transform. Native longitude zero points +X; the Mitsuba adapter
converts its -Z convention by a matrix, without resampling the image.
Checkerboard periods have four alternating quadrants per unit UV square,
with explicit UV scale. Volumes, arbitrary image material textures and other
external plugins remain outside this bounded version.

Render settings use native PT/BDPT/SPPM scattering-depth budgets, resolution,
sample count, seed, BVH/kd and box/tent/Gaussian filtering. Native Gaussian
filtering uses standard deviation 0.5 and radius 2. Source output format/depth
are retained in metadata; native PNG/HDR/EXR output is selected by the renderer.
Cross-renderer depth accounting, filtering of camera splats, sampling and
BDPT geometric-normal shading can still differ; importing data does not
promise pixel-identical reference renders.

SPPM uses a separate optional `render.sppm` group. With `integrator="sppm"`,
omitted settings become `SppmSettings(iterations=256, photons_per_stage=300000)`.
Both budgets must be positive 32-bit integers. `spp` remains the PT/BDPT budget;
it is not an alias for SPPM iterations. SPPM settings may be retained in a scene
for an explicit integrator override. CLI `--iterations` / `--photons-per-stage`
override them; those flags require SPPM, and `--spp` is rejected with SPPM.
The existing SPPM Halton sampling, fixed initial radius/alpha, glossy-ray tracing
and final per-pixel density estimate are unchanged, not a promise of the same
reconstruction-filter behavior as PT/BDPT.

```powershell
$env:GYUNITY_MITSUBA_ARCHIVES = "$TestRoot\archives"
New-Item -ItemType Directory -Force "$TestRoot\native-scenes" | Out-Null
$env:GYUNITY_TEST_OUTPUT_DIRECTORY = "$TestRoot\native-scenes"
uv run --project tools\gyscene --locked python -m unittest discover -s tools\gyscene\tests -v
```

Run test commands through a test subagent. The test output directory must
already exist under the main checkout's `Test`; temporary imports are isolated
there. No imported scene parameter is hand-transcribed into the renderer.
