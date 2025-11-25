#!/usr/bin/env python3
import argparse
import os
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent
BUILD_DIR = ROOT / "build"


def run(cmd, cwd=None):
    print("[RUN]", " ".join(map(str, cmd)))
    subprocess.run(cmd, cwd=cwd, check=True)


def clean():
    if BUILD_DIR.exists():
        print("[CLEAN] Removing build directory:", BUILD_DIR)
        shutil.rmtree(BUILD_DIR)
    else:
        print("[CLEAN] Nothing to clean (build directory not found)")


def configure(build_type):
    print(f"[CONFIGURE] Build type = {build_type}")
    BUILD_DIR.mkdir(exist_ok=True)
    run(["cmake", f"-DCMAKE_BUILD_TYPE={build_type}", ".."], cwd=BUILD_DIR)


def build(target=None, jobs=8):
    if target is None:
        print("[BUILD] Building ALL targets")
        run(["make", f"-j{jobs}"], cwd=BUILD_DIR)
    else:
        print(f"[BUILD] Building target: {target}")
        run(["make", target, f"-j{jobs}"], cwd=BUILD_DIR)


def main():
    parser = argparse.ArgumentParser(description="CMake build helper")
    parser.add_argument("--clean", action="store_true",
                        help="Remove build/ directory")
    parser.add_argument("--target", type=str, default=None,
                        help="Specific target to build (e.g., build_env)")
    parser.add_argument("--debug", action="store_true",
                        help="Build in Debug mode")
    parser.add_argument("--release", action="store_true",
                        help="Build in Release mode")
    parser.add_argument("-j", "--jobs", type=int, default=8,
                        help="Number of threads for build")
    args = parser.parse_args()

    if args.clean:
        clean()
        return

    # Determine build type
    if args.debug:
        build_type = "Debug"
    elif args.release:
        build_type = "Release"
    else:
        build_type = "Release"

    # Configure & build
    configure(build_type)
    build(target=args.target, jobs=args.jobs)


if __name__ == "__main__":
    main()
