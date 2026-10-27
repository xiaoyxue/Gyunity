"""Use the same native model with an original, reviewed Mitsuba gallery archive."""
import argparse
from pathlib import Path

from gyscene import Scene
from gyscene.mitsuba import ARCHIVES, import_archive, import_xml

parser = argparse.ArgumentParser()
parser.add_argument("input", type=Path, nargs="?", help="A supported standalone scene.xml")
parser.add_argument("--archive-dir", type=Path)
parser.add_argument("--scene", choices=ARCHIVES)
parser.add_argument("--output-root", type=Path, required=True)
parser.add_argument("--name")
parser.add_argument("--max-depth", type=int)
args = parser.parse_args()

if args.input:
    if args.archive_dir or args.scene:
        parser.error("Use a standalone XML file or an archive, not both")
    output = import_xml(args.input, args.output_root, args.name, max_depth=args.max_depth)
else:
    if not args.archive_dir or not args.scene or args.name or args.max_depth is not None:
        parser.error("Archive mode requires --archive-dir and --scene; overrides apply only to standalone XML")
    output = import_archive(args.archive_dir / (args.scene + ".zip"), args.output_root)
scene = Scene.read_json(output)
print(f"{scene.name}: {len(scene.shapes)} source shapes -> {output}")
