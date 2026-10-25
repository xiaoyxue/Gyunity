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

## Version 1

JSON uses `schema_version: 1`, `name`, `render`, `camera`, named `materials`,
ordered `shapes`, and non-rendering `metadata`. Unknown fields, invalid numbers,
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
Version 1 diffuse and conductor materials are two-sided; dielectric IORs and
reflection/transmission are explicit. Area emission is attached to a shape
independently of its material. Environment lights, volume data, textures,
single-sided opaque materials and arbitrary external plugins are not yet in
this intentionally bounded contract.

Render settings use native PT/BDPT scattering-depth budgets, resolution,
sample count, seed, BVH/kd and box/tent filtering. Source output format/depth
are retained in metadata; native PNG/HDR output is selected by the renderer.
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
