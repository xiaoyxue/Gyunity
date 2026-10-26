"""Use the real gallery geometry/lighting with explicit native material choices."""
import argparse
from pathlib import Path

from gyscene.matpreview import PROFILES, import_presets

parser = argparse.ArgumentParser()
parser.add_argument("archive", type=Path)
parser.add_argument("--output-root", type=Path, required=True)
parser.add_argument("--profile", choices=(*PROFILES, "all"), required=True)
parser.add_argument("--max-depth", type=int, required=True)
args = parser.parse_args()

# For a custom native material, use import_preview(..., exterior=your_material).
for output in import_presets(args.archive, args.output_root, profile=args.profile, max_depth=args.max_depth):
    print(output)
