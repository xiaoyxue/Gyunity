# Native scene authoring

`gyscene` is Gyunity's Python scene model, not a representation of Mitsuba's
plugin tree. Native scripts and import adapters use the same typed models,
validation, JSON serializer and generated schema. The renderer needs neither
Python nor uv once JSON and referenced assets exist.

From the worktree root:

```powershell
uv sync --project tools\gyscene --locked
uv run --project tools\gyscene --locked gyscene build tools\gyscene\examples\native_demo.py --out resources\scenes\native-demo\scene.json
uv run --project tools\gyscene --locked gyscene import-mitsuba --archive-dir D:\Code\mitsuba3-scene --scene cornell-box --output-root resources\scenes\mitsuba
uv run --project tools\gyscene --locked gyscene schema --out tools\gyscene\scene.schema.json
uv run --project tools\gyscene --locked gyscene validate resources\scenes\mitsuba\cornell-box\scene.json
```

Repeat import with `veach-mis` and `veach-bidir`. The imported Cornell scene is
named **CornellBoxStandard**. Import checks pinned archive SHA-256 values and
retains the original CC0 license, XML and all referenced OBJ files unchanged.
Supplied reference images are not copied into production assets and must not
be presented as renderer output.

## Three Python entry points

All three paths construct the same `gyscene.Scene` and serialize our native
JSON. Python code is in the repository, not hidden in a renderer executable:

| Purpose | Runnable example | Reusable module |
|---|---|---|
| Author a custom scene in Python | `examples\native_demo.py` | `gyscene.model` |
| Parse original Mitsuba XML/OBJ packages | `examples\import_mitsuba.py` | `gyscene.mitsuba` |
| Parse the supported PBRT v4 surface subset | `examples\import_pbrt.py` | `gyscene.pbrt` |
| Import original material-preview geometry with an explicit supported BSDF | `examples\import_matpreview.py` | `gyscene.matpreview` |

```powershell
uv run --project tools\gyscene --locked python tools\gyscene\examples\native_demo.py --out "D:\Code\Graphics\Gyunity\Test\custom-scene.json"
uv run --project tools\gyscene --locked python tools\gyscene\examples\import_mitsuba.py --archive-dir D:\Code\mitsuba3-scene --scene cornell-box --output-root "D:\Code\Graphics\Gyunity\Test\python-imports"
uv run --project tools\gyscene --locked python tools\gyscene\examples\import_mitsuba.py resources\scenes\mitsuba\cornell-box\scene.xml --name CornellBoxStandard --output-root "D:\Code\Graphics\Gyunity\Test\xml-imports"
uv run --project tools\gyscene --locked --extra pbrt python tools\gyscene\examples\import_pbrt.py tools\gyscene\examples\pbrt_demo.pbrt --rgb-radiance-scale 1 --output-root "D:\Code\Graphics\Gyunity\Test\python-imports"
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
uv run --project tools\gyscene --locked --extra mitsuba python tools\gyscene\examples\import_matpreview.py D:\Code\mitsuba3-scene\matpreview.zip --profile diffuse --max-depth 8 --output-root "D:\Code\Graphics\Gyunity\Test\material-preview"
```

`glass` and `copper` are also available. A custom script may pass its own
native material to `import_preview`. The real geometry/camera/UVs/normals and
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
dielectric IORs and reflection/transmission are explicit. Area emission is
attached to a shape independently of its material. One `Environment` light
may reference a linear RGB HDR/EXR map, positive scale and orthonormal
direction-only transform. Native longitude zero points +X; the Mitsuba adapter
converts its -Z convention by a matrix, without resampling the image.
Checkerboard periods have four alternating quadrants per unit UV square,
with explicit UV scale. Volumes, arbitrary image material textures and other
external plugins remain outside this bounded version.

Render settings use native PT/BDPT scattering-depth budgets, resolution,
sample count, seed, BVH/kd and box/tent/Gaussian filtering. Native Gaussian
filtering uses standard deviation 0.5 and radius 2. Source output format/depth
are retained in metadata; native PNG/HDR/EXR output is selected by the renderer.
Cross-renderer depth accounting, filtering of camera splats, sampling and
BDPT geometric-normal shading can still differ; importing data does not
promise pixel-identical reference renders.

```powershell
$env:GYUNITY_MITSUBA_ARCHIVES = "D:\Code\mitsuba3-scene"
$env:GYUNITY_TEST_OUTPUT_DIRECTORY = "D:\Code\Graphics\Gyunity\Test\native-scenes"
uv run --project tools\gyscene --locked python -m unittest discover -s tools\gyscene\tests -v
```

Run test commands through a test subagent. The test output directory must
already exist under the main checkout's `Test`; temporary imports are isolated
there. No imported scene parameter is hand-transcribed into the renderer.
