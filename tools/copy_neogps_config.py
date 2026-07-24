Import("env")
import shutil
import os

def copy_neogps_config(source, target, env):
    src_dir = os.path.join(env["PROJECT_DIR"], "lib", "gps_math")
    dst_dirs = []

    # Priority 1: .pio/libdeps/<env>/NeoGPS/src (PIO-managed copy used at compile time)
    libdeps = os.path.join(env["PROJECT_LIBDEPS_DIR"], env["PIOENV"], "NeoGPS", "src")
    if os.path.isdir(libdeps):
        dst_dirs.append(libdeps)

    # Priority 2: lib/NeoGPS/src (git submodule, also used by LDF deep+)
    for d in env.get("LIBSOURCE_DIRS", []):
        candidate = os.path.join(str(d), "NeoGPS", "src")
        if os.path.isdir(candidate):
            dst_dirs.append(candidate)
            break

    if not dst_dirs:
        print("WARNING: NeoGPS src dir not found, skipping config copy")
        return

    for cfg in ["GPSfix_cfg.h", "NMEAGPS_cfg.h", "NeoGPS_cfg.h"]:
        src = os.path.join(src_dir, cfg)
        if os.path.isfile(src):
            for dst_dir in dst_dirs:
                dst = os.path.join(dst_dir, cfg)
                shutil.copy2(src, dst)
                print(f"  Copied {cfg} -> {dst_dir}")

env.AddPreAction("buildprog", copy_neogps_config)
# Also run before lib building
env.AddPreAction("$BUILD_DIR/lib", copy_neogps_config)
