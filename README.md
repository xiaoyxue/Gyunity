# Gyunity
## Description
This is a physically based renderer (CPU based ray tracing), and it's original name is SmallSPPM since I have answered someone's question about sppm to write a demo. But now I decide to extend it to a graphics library, so I rename this project to "Gyunity" [dʒiːjuːnəti].

Next plan is to implement the GPU based renderer and add more features like MCMC(MMLT), VCM, Differentiable Rendering core, Path Guiding methods...

## Features
- PathTracing (PT)
- Stochastic progressive photon mapping (SPPM)
- LightTracing (LT)
- Bidirectional PathTracing (BDPT)
- Volume PathTracing (VPT)
- Equi-angular sampling
- Multiple importance sampling (MIS)
- Microfacet model with GGX distribution (GGX)
- Ambient Occlusion (AO)
- Low Discrepancy Sequence (Halton, Sobol...)
- KdTree, BVH
- Fast vector and matrix calculation (SIMD-SSE)
- Simple Memory Pool

## Some Results
### Original scene previews
Height 1024 with original aspect ratios, 512 samples per pixel and depth 10.
Veach bidir uses BDPT; the other five use PT. These are generated Gyunity
renders with no resizing or reference-image substitution.
[Settings and image hashes](https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/transport-scenes-1024h.json).

<div align="center">
  <img width="400" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/CornellBoxStandard-path-1024x1024-spp512-depth10.png?raw=true" alt="CornellBoxStandard, path tracing"/>
  <img width="400" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/veach-mis-path-1820x1024-spp512-depth10.png?raw=true" alt="Original Veach MIS, path tracing"/>
  <p>CornellBoxStandard (PT) / Veach MIS (PT)</p>
</div>

<br/>
<div align="center">
  <img width="400" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/veach-bidir-bdpt-1024x1024-spp512-depth10.png?raw=true" alt="Original Veach bidir, bidirectional path tracing"/>
  <img width="400" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/matpreview-copper-path-1366x1024-spp512-depth10.png?raw=true" alt="Original Matpreview geometry with native copper, path tracing"/>
  <p>Veach bidir (BDPT) / Matpreview copper (PT)</p>
</div>

<br/>
<div align="center">
  <img width="400" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/matpreview-glass-path-1366x1024-spp512-depth10.png?raw=true" alt="Original Matpreview geometry with native glass, path tracing"/>
  <img width="400" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/matpreview-diffuse-path-1366x1024-spp512-depth10.png?raw=true" alt="Original Matpreview geometry with native diffuse material, path tracing"/>
  <p>Matpreview glass (PT) / Matpreview diffuse (PT)</p>
</div>

Matpreview uses explicit supported native material overrides. Source geometry
and environments are not redistributed here. Some Monte Carlo noise,
fireflies and caustic speckles remain.

### Earlier renders
<div align="center">
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/bunny2.png?raw=true"/>
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/bunny.png?raw=true"/>
</div>

<br/>
<div align="center">
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/coca1.png?raw=true"/>
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/coca2.png?raw=true"/>
</div>

<br/>
<div align="center">
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/EiffelTower_GGX_RoughGlass_EnvMap.png?raw=true"/>
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/EiffelTower_Glass_EnvMap.png?raw=true"/>
</div>

<br/>
<div align="center">
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/Lucy_Bunny.png?raw=true"/>
  <img width="400px" height="400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/water_cornell_box.png?raw=true"/>
</div>

<br/>
<div align="center">
  <img style = "width: 400px; height: 400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/bunny_envmap_scale_down_1080.gif?raw=true" loop>
  <img style = "width: 400px; height: 400px" src="https://github.com/xiaoyxue/GyunityAssets/blob/main/GyunityAssets/Gallery/sppm_cornell_box_area_light_scale_down_1080.gif?raw=true" loop>
</div>

## Build
Ensure CMake is on `PATH`. Visual Studio 2026 requires CMake 4.2 or newer.

1. Clone the repository: `git clone --recursive https://github.com/xiaoyxue/Gyunity.git`
2. From the repository directory, run `.\init.ps1` in Developer PowerShell for VS, or `init.cmd` in Developer Command Prompt for VS. This recreates `external\build` and builds the dependencies in Debug and Release.
3. Open `Gyunity.sln` and build the solution.

## Run
After building Release x64, run these commands from the repository/worktree root:

```powershell
$TestRoot = Join-Path (Resolve-Path (Join-Path (git --no-pager rev-parse --path-format=relative --git-common-dir) '..')).Path 'Test'
New-Item -ItemType Directory -Force $TestRoot | Out-Null
Set-Location GyunityApp
..\x64\Release\GyunityApp.exe --list-scenes
..\x64\Release\GyunityApp.exe --scene CornellBoxStandard --integrator path --width 256 --height 256 --spp 16 --max-depth 8 --out "$TestRoot\CornellBoxStandard.png"
```

Use `--integrator bdpt` for BDPT, `--help` for options, or `--list-examples` to
list callable examples. With no arguments, the original PT/environment demo
runs. Outputs belong in the main checkout's `Test`, including from worktrees.
Matpreview requires the one-time local import described in the scene documentation.
See [INSTRUCTIONS.md](INSTRUCTIONS.md) for cmd.exe, Python/uv, PowerShell and SPPM commands.

## Documentation

| Topic | Details |
|---|---|
| [Projects](Docs/Projects.md) | Repository and application layout |
| [Scene authoring](Docs/SceneAuthoring.md) | Native Python/JSON, uv and Mitsuba/PBRT imports |
| [Scene builder](Docs/SceneBuilder.md) | C++ construction API and legacy scene correspondence |
| [Scenes](Docs/Scenes.md) | Named scene commands, Cornell/Veach data and Matpreview preparation |
| [Examples](Docs/Examples.md) | C++ helpers, named examples and explicit batch rendering |
| [BDPT](Docs/BDPT.md) | Probability interfaces, subpaths, rendering and limitations |
| [Debugging](Docs/Debugging.md) | Thread-local pixel diagnostics |
| [Testing](Docs/Testing.md) | Scene regression commands and reference comparisons |

Paths in the detailed documentation are repository-relative unless stated otherwise.
