# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Render registered scenes and compare exact PNG hashes, without editing sources."""

import argparse
import csv
import hashlib
from pathlib import Path
import subprocess
import sys


def digest(path: Path) -> str:
    with path.open("rb") as image:
        return hashlib.file_digest(image, "sha256").hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--reference-directory", type=Path, required=True)
    parser.add_argument("--configuration", choices=("Debug", "Release"), default="Release")
    parser.add_argument("--scenes", nargs="+", help="Subset of registered names; default: all")
    args = parser.parse_args()

    root = Path(__file__).resolve().parent.parent
    executable = root.joinpath("x64", args.configuration, "GyunityApp.exe")
    working_directory = root.joinpath("GyunityApp")
    output = args.output_directory.resolve()
    reference = args.reference_directory.resolve()
    if not executable.is_file():
        parser.error(f"Build {args.configuration}|x64 first: {executable}")
    if not reference.is_dir():
        parser.error(f"Reference directory does not exist: {reference}")
    if output == reference:
        parser.error("Use separate output and reference directories.")

    listed = subprocess.run(
        [str(executable), "--list-scenes"], cwd=working_directory,
        capture_output=True, text=True, check=False,
    )
    if listed.returncode != 0:
        print(listed.stderr, file=sys.stderr)
        return listed.returncode
    registered = listed.stdout.splitlines()
    if not registered:
        parser.error("The application returned an empty scene registry.")
    scenes = args.scenes or registered
    for scene in scenes:
        if scene not in registered:
            parser.error(f"Unknown registered scene: {scene}")
        for accelerator in ("bvh", "kd"):
            baseline = reference.joinpath(f"old.{scene}.{accelerator}.png")
            if not baseline.is_file():
                parser.error(f"Missing reference: {baseline}")

    output.mkdir(parents=True, exist_ok=True)
    matched = True
    with output.joinpath("comparisons.csv").open("w", newline="", encoding="utf-8") as report:
        writer = csv.DictWriter(report, fieldnames=("Scene", "Accelerator", "Identical"))
        writer.writeheader()
        for scene in scenes:
            for accelerator in ("bvh", "kd"):
                image = output.joinpath(f"registry.{scene}.{accelerator}.png")
                log = output.joinpath(f"registry.{scene}.{accelerator}.log")
                command = [
                    str(executable), "--scene", scene, "--accel", accelerator,
                    "--out", str(image), "--width", "512", "--height", "512",
                    "--spp", "64", "--max-depth", "8",
                ]
                with log.open("wb") as stream:
                    rendered = subprocess.run(
                        command, cwd=working_directory, stdout=stream,
                        stderr=subprocess.STDOUT, check=False,
                    )
                if rendered.returncode != 0:
                    print(f"Render failed: {scene}/{accelerator}; see {log}", file=sys.stderr)
                    return rendered.returncode
                baseline = reference.joinpath(f"old.{scene}.{accelerator}.png")
                identical = digest(image) == digest(baseline)
                writer.writerow({"Scene": scene, "Accelerator": accelerator, "Identical": identical})
                report.flush()
                print(f"{scene}/{accelerator} identical={identical}", flush=True)
                matched = matched and identical
    if not matched:
        print("Scene regression failed: PNG bytes differ.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except OSError as error:
        print(f"Scene regression I/O failure: {error}", file=sys.stderr)
        sys.exit(1)
