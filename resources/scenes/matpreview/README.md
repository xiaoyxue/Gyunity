# Original material-preview import (local assets)

The Mitsuba gallery `matpreview.zip` is a different asset from LTE Orb. It
contains no license file, so this repository does **not** redistribute its
raw/derived geometry, EXR environment or supplied reference renders. Use the
user-provided local archive; do not infer CC0 from the other gallery packages.

From the worktree root:

```powershell
uv run --project tools\gyscene --locked --extra mitsuba gyscene import-matpreview D:\Code\mitsuba3-scene\matpreview.zip --profile diffuse --max-depth 8 --output-root "D:\Code\Graphics\Gyunity\Test\material-preview"
```

Use `glass` or `copper` for other supported **explicit material overrides**.
`diffuse` retains the source green diffuse color; `glass` retains the source
IOR; `copper` is a documented native conductor demonstration preset, not
source plastic. The original plastic BSDF is not silently approximated.
A custom Python script can pass its own native material to `import_preview`.

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

Render the generated JSON with the native app (no Python runtime needed):

```powershell
.\x64\Release\GyunityApp.exe --scene-file "D:\Code\Graphics\Gyunity\Test\material-preview\matpreview\matpreview-diffuse.json" --integrator bdpt --width 683 --height 512 --spp 128 --out "D:\Code\Graphics\Gyunity\Test\material-preview\matpreview-diffuse-bdpt.exr"
```

Choose `.png` for a display preview. Generated images must be labeled as
Gyunity results and not confused with the archive's supplied images.
