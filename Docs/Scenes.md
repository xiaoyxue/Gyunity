# Named scenes and rendering

`GyunityApp\scenes\SceneRegistry.h` maps names to builders, default cameras and required
resources. Names are `mesh`, `slope`, `sphere`, `env`, `triangle`, `textured`,
`empty`, `diffuse`, `water`, `lucy-bunny`, `fog`, `CornellBoxStandard`, `veach-mis`,
and `veach-bidir`, plus the nine `matpreview-*` entries listed in the
[local preparation recipe](../resources/scenes/matpreview/README.md).
Matpreview names are listed even before their local assets are imported;
trying to render one without assets reports the preparation command.
The sphere and triangle scenes use their large-room camera; the imported
Cornell and Veach scenes use their source cameras, and other builders use the standard Cornell camera. Fog automatically
uses volume path tracing. `veach-bidir` defaults to BDPT; other supplied scenes
default to surface path tracing. Native JSON supplies its own render defaults.

```powershell
Set-Location GyunityApp
$TestRoot = Join-Path (Resolve-Path (Join-Path (git --no-pager rev-parse --path-format=relative --git-common-dir) '..')).Path 'Test'
..\x64\Release\GyunityApp.exe --list-scenes
..\x64\Release\GyunityApp.exe --scene mesh --out "$TestRoot\mesh.png"
..\x64\Release\GyunityApp.exe --scene fog --accel kd --spp 128 --width 640 --height 360 --out "$TestRoot\fog.png"
```

Defaults are 512x512, 64 samples per pixel, depth 8, and BVH. Use `--width`,
`--height`, `--spp`, `--max-depth`, and `--accel bvh|kd` to override them.
The sampler setup matches the regression harness (`RandomSampler(123)` with
the renderer's existing per-tile cloning). `--out` is required and accepts
lowercase `.png`, `.bmp`, `.hdr` or `.exr`; output directories must already exist.
Paths with spaces must be quoted. `--help` and `--list-scenes` are standalone
commands. Invalid options, missing assets and failed image writes report an
error and return a nonzero exit code; no image is silently substituted.

Surface integrator overrides are `--integrator path|bdpt|sppm`. SPPM uses
`--iterations` (default 256) and `--photons-per-stage` (default 300000);
`--spp` is rejected for SPPM, and its budget flags are rejected for PT/BDPT.
The existing SPPM implementation retains Halton sampling, radius 0.05,
alpha 0.66666667, per-hit-point shrinking and glossy-ray tracing. Its final
per-pixel density estimate is not the PT/BDPT reconstruction-filter estimator.
These differences and the existing SPPM accumulation behavior are not changed
by exposing it through the scene interface.

## Original Veach MIS
`veach-mis` loads the actual CC0 gallery data, not a procedural recreation.
The four solid bars retain their source transforms and Beckmann alpha values
0.01, 0.05, 0.1 and 0.25. The **three** sphere emitters retain their source
positions, radii (1, 0.5 and 0.05), radiances, and approximately equal power.
Original complex IORs, reflectance, camera and 1280x720 aspect ratio are
preserved by the Python importer and native JSON loader.

```powershell
# From GyunityApp; use the same arguments with --integrator path for PT.
..\x64\Release\GyunityApp.exe --scene veach-mis --integrator bdpt --width 512 --height 288 --spp 128 --max-depth 3 --out "$TestRoot\veach-mis-bdpt.png"
```

The importer and source license are under `resources\scenes\mitsuba\veach-mis`.
`VeachMisScene.*` checks the real geometry/light counts, radiance/power,
camera orientation and raw-alpha conductor response; it does not compare
against a fabricated scene.

## Original Veach bidir
`veach-bidir` loads the original gallery XML-to-JSON conversion and **all 12
unaltered OBJ files**: the room, floor lamp, wall-mounted light, table and
glass object. It is not the removed procedural room substitute. Original
Beckmann lamp materials, glass IORs, per-mesh normal settings, emitting
rectangle BSDFs/radiances and the source camera are retained.

```powershell
# From GyunityApp; use --integrator path for the PT comparison.
..\x64\Release\GyunityApp.exe --scene veach-bidir --integrator bdpt --width 512 --height 512 --spp 128 --max-depth 8 --out "$TestRoot\veach-bidir-bdpt.png"
```

The source default depth is 7; the command explicitly overrides it to 8.
PT may have considerable noise/fireflies at this sample budget; changing the
materials or light intensity to hide that would invalidate the import.
BDPT uses geometric-normal shading by default even when the imported OBJ has
smooth normals; the faceted glass then produces lattice-like caustics. Add
`--bdpt-normals shading` for adjoint-corrected smooth normals (see
[BDPT](BDPT.md#smooth-shading-normals)). `VeachBidirScene.*` checks primitive counts against the actual
OBJ faces, source normal policies, camera conversion and nonblack diffuse
BSDFs on the emitting rectangles.

LDR outputs accept PBRT `imgtool` tone mapping with 8-bit sRGB encoding, and
`--convert` applies it to an existing image:

```powershell
..\x64\Release\GyunityApp.exe --scene veach-bidir --integrator bdpt --bdpt-normals shading --spp 256 --out "$TestRoot\veach-bidir.exr"
..\x64\Release\GyunityApp.exe --convert "$TestRoot\veach-bidir.exr" --tonemap aces --out "$TestRoot\veach-bidir-aces.png"
..\x64\Release\GyunityApp.exe --convert "$TestRoot\veach-bidir.exr" --tonemap reinhard --max-luminance 4 --out "$TestRoot\veach-bidir-reinhard.png"
```

`aces` is pbrt-v4's `--acesfilmic` (Narkowicz fit per channel, no implicit exposure);
`reinhard` is pbrt-v3's `--tonemap` with `--max-luminance` as the white luminance.
`--tonemap-scale` multiplies values first, like pbrt's `--scale`.

The named scene and C++ `Examples::VeachBidir` helper default to **BDPT**.
The reviewed archive importer records this native integrator override in
metadata; the original source XML remains unchanged. `--integrator path`
explicitly selects PT for comparisons.
