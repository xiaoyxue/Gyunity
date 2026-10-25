"""Use the real gallery geometry/lighting with explicit native material choices."""
import argparse
from pathlib import Path

from gyscene.matpreview import PROFILES, import_preview, profile_material, source_data

parser = argparse.ArgumentParser()
parser.add_argument("archive", type=Path)
parser.add_argument("--output-root", type=Path, required=True)
parser.add_argument("--profile", choices=PROFILES, required=True)
parser.add_argument("--max-depth", type=int, required=True)
args = parser.parse_args()

root, _ = source_data(args.archive)
material = profile_material(args.profile, root)
# A custom script may instead pass any supported Diffuse, Conductor or Dielectric.
output = import_preview(args.archive, args.output_root, profile_name=args.profile,
                        exterior=material, max_depth=args.max_depth)
print(output)
