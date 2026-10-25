import argparse
import json
from pathlib import Path
import runpy
import sys
from zipfile import BadZipFile
from xml.etree.ElementTree import ParseError

from .model import Scene
from .mitsuba import ARCHIVES, import_archive


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
        else:
            print(import_archive(args.archive_dir / (args.scene + ".zip"), args.output_root))
    except (OSError, ValueError, KeyError, BadZipFile, ParseError) as error:
        print(f"gyscene: {error}", file=sys.stderr)
        return 1
    return 0
