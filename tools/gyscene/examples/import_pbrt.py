"""Parse PBRT v4 source data into the same native Scene used by native_demo.py."""
import argparse
from pathlib import Path

from gyscene import Scene
from gyscene.pbrt import import_file

parser = argparse.ArgumentParser()
parser.add_argument("input", type=Path)
parser.add_argument("--output-root", type=Path, required=True)
parser.add_argument("--rgb-radiance-scale", type=float, required=True,
                    help="Explicit RGB photometric unit conversion, not a spectral rendering match")
args = parser.parse_args()

output = import_file(args.input, args.output_root, rgb_radiance_scale=args.rgb_radiance_scale)
scene = Scene.read_json(output)
print(f"{scene.name}: {len(scene.shapes)} source shapes -> {output}")
