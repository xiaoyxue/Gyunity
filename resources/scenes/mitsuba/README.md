# Original Mitsuba gallery scene data

These packages come from the [Mitsuba simple-scene gallery](https://mitsuba.readthedocs.io/en/stable/src/gallery.html#simple-scenes),
which credits Benedikt Bitterli. Each package retains its original CC0
`LICENSE.txt`, `scene.xml`, and required OBJ files. `scene.json` is generated
by `tools\gyscene` through the same native model used for Python authoring.
The original files are not edited, and no geometry or scene parameters are
hand-transcribed into C++.

Reproduce an import from the worktree root:

```powershell
uv run --project tools\gyscene --locked gyscene import-mitsuba --archive-dir D:\Code\mitsuba3-scene --scene cornell-box --output-root resources\scenes\mitsuba
```

Use `veach-mis` or `veach-bidir` for the other reviewed source ZIPs.
Archive hashes are pinned by the importer and stored in generated metadata.
Git attributes preserve original XML, OBJ and license bytes without line-ending
normalization; generated JSON uses LF line endings.
The Cornell scene's native name is `CornellBoxStandard`. Paths to OBJ assets
are relative to each JSON file.

The archives' `TungstenRender.png` and `.exr` are supplied **Tungsten reference
images**, not Gyunity output, and are deliberately omitted from runtime
resources. Keep any comparison copies and actual Gyunity renders under the
main checkout's `Test`. Importing the original data does not imply identical
samplers, path-depth accounting, shading-normal transport or tone mapping
between renderers.
