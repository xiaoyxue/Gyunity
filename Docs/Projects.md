# Projects

| Project | What it is |
|---|---|
| `Gyunity` | The renderer itself, built as a static library: integrators, BSDFs and materials, lights, shapes, accelerators, samplers and math. |
| `GyunityApp` | The renderer application (`x64\<Config>\GyunityApp.exe`). Use `--scene` for registered scenes, `--example` for application examples, or no arguments for the original PathTracing demo. Run from `GyunityApp` so `..\resources` paths resolve. |
| `GyunityTest` | Unit tests (googletest). |

The app keeps `main.cpp`, `Examples.*`, `SceneCommandLine.*` and `SceneRenderer.*`
at its root. `Run` and entry-point routing live in `main.cpp`; Examples and the
CLI independently call the shared renderer without depending on each other. Builder-based
scene headers live in `GyunityApp\scenes`; the unchanged original scene header
and entry point live in `GyunityApp\legacy`. Tracked `.vcxproj.filters` mirror
these app folders in Visual Studio and group `GyunityTest`'s existing `tests`
folder and shared application source. Filters organize Solution Explorer only;
compilation and legacy exclusion remain controlled by the `.vcxproj` files.
