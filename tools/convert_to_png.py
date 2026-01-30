#!/usr/bin/env python3
"""
Convert JPEG map tiles to PNG format.
This creates PNG versions while keeping the original JPEG files.
"""

import os
import sys
from PIL import Image
from pathlib import Path
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed

def convert_jpeg_to_png(jpeg_path, keep_jpeg=True):
    """Convert a JPEG tile to PNG format."""
    try:
        img = Image.open(jpeg_path)

        png_path = str(jpeg_path).replace(".jpg", ".png").replace(".jpeg", ".png")

        img.save(png_path, "PNG", optimize=True)

        if not keep_jpeg:
            os.remove(jpeg_path)

        return True, jpeg_path
    except Exception as e:
        return False, f"{jpeg_path}: {e}"

def convert_directory(base_path, keep_jpeg=True, jobs=4):
    """Convert all JPEG tiles in a directory tree."""
    base = Path(base_path)

    if not base.exists():
        print(f"Error: {base_path} does not exist")
        return

    # Find all JPEG files (both .jpg and .jpeg extensions)
    jpeg_files = list(base.rglob("*.jpg")) + list(base.rglob("*.jpeg"))
    total = len(jpeg_files)

    if total == 0:
        print(f"No JPEG files found in {base_path}")
        return

    print(f"Found {total} JPEG tiles to convert")
    print(f"Keep JPEG files: {keep_jpeg}")
    print(f"Parallel jobs: {jobs}")
    print()

    converted = 0
    errors = 0

    with ThreadPoolExecutor(max_workers=jobs) as executor:
        futures = {
            executor.submit(convert_jpeg_to_png, jpeg_path, keep_jpeg): jpeg_path
            for jpeg_path in jpeg_files
        }

        for i, future in enumerate(as_completed(futures)):
            success, result = future.result()
            if success:
                converted += 1
            else:
                errors += 1
                print(f"  Error: {result}")

            if (i + 1) % 500 == 0 or i == 0 or i == total - 1:
                print(f"Progress: {i+1}/{total} ({100*(i+1)//total}%)")

    print()
    print(f"Done! Converted: {converted}, Errors: {errors}")

    # Show space comparison
    jpeg_size = sum(f.stat().st_size for f in base.rglob("*.jpg")) + sum(f.stat().st_size for f in base.rglob("*.jpeg"))
    print(f"JPEG total: {jpeg_size / 1024 / 1024:.1f} MB")

    png_size = sum(f.stat().st_size for f in base.rglob("*.png"))
    print(f"PNG total: {png_size / 1024 / 1024:.1f} MB")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert JPEG map tiles to PNG format")
    parser.add_argument("path", help="Path to directory containing JPEG tiles")
    parser.add_argument("-j", "--jobs", type=int, default=4,
                        help="Number of parallel conversion jobs (default: 4)")
    parser.add_argument("--no-keep-jpeg", action="store_false", dest="keep_jpeg",
                        help="Remove JPEG files after conversion (default: keep JPEG files)")

    args = parser.parse_args()

    convert_directory(args.path, args.keep_jpeg, args.jobs)