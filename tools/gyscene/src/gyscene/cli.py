import argparse
import json
from pathlib import Path
import runpy
import sys
from zipfile import BadZipFile
from xml.etree.ElementTree import ParseError

from .model import Scene
from .mitsuba import ARCHIVES, import_archive, import_xml
from .pbrt import import_file as import_pbrt
from .matpreview import PROFILES, import_preview, profile_material, source_data


def main() -> int:
    parser = argparse.ArgumentParser(prog="gyscene")
    commands = parser.add_subparsers(dest="command", required=True)
    build = commands.add_parser("build", help="Run a trusted native Python scene script")
    build.add_argument("script", type=Path)
    build.add_argument("--out", type=Path, required=True)
    validate = commands.add_parser("validate")
    validate.add_argument("scene", type=Path)
    schema = commands.add_parser("schema")
    schema.add_argument("--out", type=Path, required=True)
    importer = commands.add_parser("import-mitsuba")
    importer.add_argument("--archive-dir", type=Path, required=True)
    importer.add_argument("--scene", choices=list(ARCHIVES), required=True)
    importer.add_argument("--output-root", type=Path, required=True)
    xml = commands.add_parser("import-mitsuba-file", help="Import the supported standalone Mitsuba XML subset")
    xml.add_argument("scene", type=Path)
    xml.add_argument("--output-root", type=Path, required=True)
    xml.add_argument("--name")
    xml.add_argument("--max-depth", type=int)
    pbrt = commands.add_parser("import-pbrt", help="Import the documented static RGB PBRT v4 subset")
    pbrt.add_argument("scene", type=Path)
    pbrt.add_argument("--output-root", type=Path, required=True)
    pbrt.add_argument("--name")
    pbrt.add_argument("--rgb-radiance-scale", type=float, required=True,
                      help="Explicit native RGB unit scale after photometric normalization (not PBRT spectral calibration)")
    preview = commands.add_parser("import-matpreview", help="Locally import original matpreview geometry with an explicit supported material")
    preview.add_argument("archive", type=Path)
    preview.add_argument("--output-root", type=Path, required=True)
    preview.add_argument("--profile", choices=PROFILES, required=True)
    preview.add_argument("--max-depth", type=int, required=True)
    args = parser.parse_args()
    try:
        if args.command == "build":
            scene = runpy.run_path(str(args.script)).get("scene")
            if not isinstance(scene, Scene):
                raise ValueError("The Python script must export a gyscene.Scene named 'scene'")
            scene.write_json(args.out)
            print(args.out)
        elif args.command == "validate":
            Scene.read_json(args.scene)
            print(f"Valid native scene: {args.scene}")
        elif args.command == "schema":
            args.out.parent.mkdir(parents=True, exist_ok=True)
            args.out.write_text(json.dumps(Scene.model_json_schema(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
            print(args.out)
        elif args.command == "import-mitsuba":
            print(import_archive(args.archive_dir / (args.scene + ".zip"), args.output_root))
        elif args.command == "import-mitsuba-file":
            print(import_xml(args.scene, args.output_root, args.name, max_depth=args.max_depth))
        elif args.command == "import-pbrt":
            print(import_pbrt(args.scene, args.output_root, args.name, rgb_radiance_scale=args.rgb_radiance_scale))
        else:
            root, _ = source_data(args.archive)
            print(import_preview(args.archive, args.output_root, profile_name=args.profile,
                                 exterior=profile_material(args.profile, root), max_depth=args.max_depth))
    except (OSError, ValueError, KeyError, BadZipFile, ParseError) as error:
        print(f"gyscene: {error}", file=sys.stderr)
        return 1
    return 0
