# Native Python scene authoring

`tools\gyscene` provides a uv-managed native scene model, versioned JSON and
generated JSON Schema. Python-authored scenes and the original Mitsuba gallery
importer use the same model and serializer; the format is not XML-shaped JSON.
See [the tool documentation](../tools/gyscene/README.md) for authoring, import and
validation commands. The app loads the native JSON directly, with no Python
dependency at render time.

The importer accepts the original `cornell-box.zip`, `veach-mis.zip` and
`veach-bidir.zip`, verifies source hashes, and preserves XML, CC0 licenses and
OBJ data. The imported Cornell scene is named `CornellBoxStandard`.
No gallery geometry, camera or material values are manually transcribed.

The repository also includes directly runnable Python examples:
`tools\gyscene\examples\native_demo.py` authors a custom scene,
`import_mitsuba.py` parses either a supported XML file or a reviewed archive,
and `import_pbrt.py` parses the documented PBRT v4 surface subset. All three
produce the same native JSON. Use `uv run --project tools\gyscene --locked`
with these scripts; PLY support uses the optional `pbrt` dependency group.
The PBRT adapter requires an explicit RGB photometric unit conversion and
rejects unsupported spectral/material/scene features rather than guessing.

From `GyunityApp`, render either a file or the imported Cornell registry entry:

```powershell
$TestRoot = Join-Path (Resolve-Path (Join-Path (git --no-pager rev-parse --path-format=relative --git-common-dir) '..')).Path 'Test'
..\x64\Release\GyunityApp.exe --scene-file ..\resources\scenes\native-demo\scene.json --out "$TestRoot\native-demo.png"
..\x64\Release\GyunityApp.exe --scene CornellBoxStandard --integrator bdpt --width 512 --height 512 --spp 128 --max-depth 8 --out "$TestRoot\CornellBoxStandard-bdpt.png"
```

Explicit flags override JSON render defaults; legacy named scenes keep their
existing defaults. `--scene` and `--scene-file` are mutually exclusive.
The native loader preserves the original area-light BSDF as well as emission,
supports box/tent/Gaussian filtering and raw-alpha GGX/Beckmann conductors, and rejects
unsupported or malformed input. Geometry and camera conventions are defined
in the tool documentation. Native mesh paths resolve relative to the JSON.
The JSON parser is the pinned `external\json` submodule; update submodules
before building. Application/test projects use C++17 for filesystem support.

Native integrators include PT, BDPT and SPPM. SPPM uses independent
`render.sppm.iterations` / `photons_per_stage` settings and CLI
`--iterations` / `--photons-per-stage` overrides, not `--spp`.
See [command instructions](../INSTRUCTIONS.md) for complete examples.

The imported `CornellBoxStandard` is additional to the unchanged legacy Cornell
demos. Its source camera, red/green walls, two transformed boxes, tent filter
and colored emitter are taken from the CC0 gallery XML. PNG/HDR files generated
by Gyunity are distinct from the archive's supplied Tungsten reference images.
BDPT uses geometric-normal shading by default (smooth normals are opt-in with
`--bdpt-normals shading`) and unfiltered camera splats;
those existing limitations are not claims of exact cross-renderer equivalence.

C++ failures use checked statuses, not exception-based control flow:
`ParseSceneCommandLine` returns bool and fills output options on success,
`FindScene`/`LoadNativeScene` return nullable pointers, and rendering/example
functions propagate bool results. Image loading uses bool output-parameter
APIs; callers must check them. EXR I/O uses OpenEXRCore result codes and currently
supports single-part, flat scanline RGB HALF/FLOAT input and Float32 output.
Unsupported EXR layouts return an error rather than being approximated.
Python retains normal validation exceptions.
Only the outermost application `main` boundary catches unexpected standard
library/dependency exceptions, reports them, and exits with failure; ordinary
validation and I/O errors still use explicit statuses.

Material-preview support uses the original local `matpreview.zip`, with nine
explicit profiles covering all six existing native material families, including
mirror, rough glass and native plastic. These replace the source exterior
explicitly; native plastic uses Mitsuba-style smooth layering and source IORs.
The Python importer preserves real geometry, camera, checkerboard
and EXR lighting; no approximate replacement mesh is created.
See [the local import recipe](../resources/scenes/matpreview/README.md).
The archive does not include a license, so its raw/derived assets remain
outside repository history. OpenEXR and Imath are pinned codec dependencies;
they do not replace Gyunity's own vector/matrix or array types.
