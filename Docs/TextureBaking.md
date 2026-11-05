# Planar texture baking

Texture baking pre-renders an image on a flat, textured plane with Gyunity's
path tracer. The camera and a matching-aspect rectangular area light are
constructed from the input image. This is a fixed-lighting appearance preset,
not mesh UV-atlas baking.

## Command line

Build Release x64 as described in [the command instructions](../INSTRUCTIONS.md).
From the checkout or worktree root:

```powershell
$TestRoot = Join-Path (Resolve-Path (Join-Path (git --no-pager rev-parse --path-format=relative --git-common-dir) '..')).Path 'Test'
New-Item -ItemType Directory -Force $TestRoot | Out-Null
.\x64\Release\GyunityApp.exe --bake-texture "D:\Assets\logo.png" `
  --roughness 0.02 --spp 256 --out "$TestRoot\logo-baked.png"
```

Input and output paths are absolute or relative to the current working
directory, not to `resources`. The command does not require any bundled scene
assets. Output directories must already exist.

| Option | Meaning | Default |
|---|---|---|
| `--bake-texture FILE` | Required 8-bit source image | None |
| `--out FILE.png` | Required RGBA PNG output | None |
| `--roughness R` | Finite artist roughness in `(0, 1]` | `0.02` |
| `--spp N` | Positive samples per pixel | `256` |
| `--max-depth N` | Positive scattering-event budget | `5` |
| `--accel bvh\|kd` | Scene accelerator | `bvh` |

The original resolution, aspect ratio and full canvas are retained, including
transparent padding. `--width` and `--height` are rejected rather than changing
the image framing. Baking always uses PT; scene selection, other integrators,
SPPM/BDPT settings, conversion and tone-mapping options cannot be combined with
`--bake-texture`. The dynamic input-image scene is not a `--scene` registry entry.

For example, change only roughness while keeping the sampling budget fixed:

```powershell
foreach ($roughness in @("0.02", "0.06", "0.12", "0.15")) {
  .\x64\Release\GyunityApp.exe --bake-texture "D:\Assets\logo.png" `
    --roughness $roughness --spp 256 --out "$TestRoot\logo-roughness-$roughness.png"
  if ($LASTEXITCODE -ne 0) { break }
}
```

## Scene and material

For input aspect ratio `a = width / height`:

| Component | Configuration |
|---|---|
| Image plane | Width `2a`, height `2`, centered at `(0, 0, 1)`, normal `+Z` |
| Camera | Position `(0, 0, 3)`, look-at `(0, 0, 0)`, up `+Y` |
| Projection | Vertical FOV approximately `53.130102` degrees, film distance `1` |
| Area light | Width `16a`, height `16`, centered at `(0, 0, 10)`, normal `-Z` |
| Emission | RGB radiance `(7.5, 7.5, 7.5)` |
| Geometry | Two image-plane triangles and two emissive triangles |
| Environment | None |

Each light side is eight times the corresponding image-plane side. The
emission value is radiance, not total light power. The camera covers the whole
image plane without cropping.

The surface uses `RoughnessMaterial` with GGX reflection:

- Reflectance multiplier: white `(1, 1, 1)`.
- Exterior IOR: `(1, 1, 1)`; interior `eta`: `(1.25, 1.25, 1.25)`.
- Extinction coefficient `k`: the decoded image texture.
- Equal horizontal and vertical roughness, using the existing
  `GGXDistribution::RoughnessToAlpha` mapping and visible-normal sampling.

Artist roughness is not raw GGX alpha: `0.02` maps to approximately `0.19253`,
and `0.15` maps to approximately `0.58135`.

The image controls **k, not diffuse albedo or the reflectance multiplier**.
Consequently, black input can still reflect light, and the rendered colors need
not match the source colors. This is not a color-preserving diffuse preset.

## Color and transparency

The CLI accepts stb-supported 8-bit images, including PNG, JPEG and BMP.
Grayscale inputs expand to RGB; images without alpha become opaque. HDR,
EXR and 16-bit-per-channel input are not supported by this command.

Input RGB is decoded with power gamma `2.2`, matching the existing LDR texture
convention. Bilinear sampling uses alpha-weighted RGB before dividing by the
filtered alpha, so hidden RGB in fully transparent pixels does not bleed into
visible edges. Alpha itself remains linear.

After rendering, RGB is clamped and encoded with power gamma `2.2`; the
original 8-bit alpha channel is copied unchanged. Fully transparent output
pixels have zero RGB. There is no exposure normalization, tone mapping or
denoising. The PNG is straight-alpha, not premultiplied-alpha.

Alpha is an output mask, not a transmissive material or cutout geometry.
Use the baked PNG as an unlit/emissive texture when the baked appearance should
be preserved; lighting it again as ordinary diffuse albedo adds illumination
on top of the baked result.

Missing/corrupt images, fully transparent inputs, invalid budgets or roughness,
unsupported formats and write failures return a nonzero process exit code.

## C++ API

The application renderer is independent of CLI parsing:

```cpp
#include "SceneRenderer.h"

bool BakeLogo(const std::string& input, const std::string& output) {
    TextureBakingOptions options;
    options.input = input;
    options.output = output;
    options.roughness = 0.02f;
    options.spp = 256;
    return RenderTextureBaking(options);
}
```

Pass an output path under the main checkout's `Test` directory for repository
experiments. `Examples::OutputPath` can construct such a path for application
examples and tests.

The lower-level handcrafted scene method is
`CornellBoxScenes::TextureBaking(builder, image, roughness)`, where `image` is
a checked `std::shared_ptr<ImageTexture<Vec3>>`. It derives the plane and light
sizes from `image->Width()` and `image->Height()` and returns `false` for invalid
inputs before adding geometry.

```cpp
auto image = std::make_shared<ImageTexture<Vec3>>(inputFilename);
auto scene = std::make_shared<Scene>();
SceneBuilder builder(scene);
if (!CornellBoxScenes::TextureBaking(builder, image, 0.02f)) return false;
auto film = std::make_shared<Film>(image->Width(), image->Height());
auto camera = SceneCamera{}.Create(film);
```

As with other builder methods, low-level callers initialize acceleration and
perform rendering themselves. Ordinary `ImageTexture<Vec3>` loading is RGB-only;
use `RenderTextureBaking` for the alpha-aware loading and RGBA output path.
`SceneBuilder::AddTexturedQuad` supplies the two triangles with corner UVs
`(0,0), (1,0), (1,1), (0,1)`.

## Focused checks

The existing test files cover scene proportions, camera/UV alignment,
texture-to-k wiring, roughness overrides, CLI validation, alpha preservation,
hidden-RGB filtering and I/O failure propagation.

```powershell
$env:GYUNITY_TEST_OUTPUT_DIRECTORY = Join-Path $TestRoot 'texture-baking-tests'
New-Item -ItemType Directory -Force $env:GYUNITY_TEST_OUTPUT_DIRECTORY | Out-Null
Push-Location GyunityApp
..\x64\Release\GyunityTest.exe --gtest_filter="CornellBoxScenes.TextureBaking*:SceneCommandLine.TextureBaking*:MainExampleRenders.TextureBaking*"
Pop-Location
```
