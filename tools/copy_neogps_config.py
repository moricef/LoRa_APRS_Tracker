Import("env")
import shutil
import os

# Copies the project NeoGPS configuration (lib/gps_math/*_cfg.h) into the
# NeoGPS sources. A "pre:" script runs when the configuration is loaded, so the
# copy is done right away, before any source or library is compiled.
# Contributed by F4INI: a pre-action on "buildprog" ran only before linking,
# after the sources had been compiled with the default NeoGPS configuration.


def copy_neogps_config():
    src_dir = os.path.join(env.subst("$PROJECT_DIR"), "lib", "gps_math")
    candidates = [
        # PlatformIO-managed copy (NeoGPS declared in lib_deps)
        os.path.join(env.subst("$PROJECT_LIBDEPS_DIR"), env.subst("$PIOENV"), "NeoGPS", "src"),
        # Copy shipped in the repository
        os.path.join(env.subst("$PROJECT_DIR"), "lib", "NeoGPS", "src"),
    ]
    dst_dirs = [d for d in candidates if os.path.isdir(d)]

    if not dst_dirs:
        print("*** ERROR: NeoGPS sources not found (lib/NeoGPS/src missing or empty).")
        env.Exit(1)

    for cfg in ["GPSfix_cfg.h", "NMEAGPS_cfg.h", "NeoGPS_cfg.h"]:
        src = os.path.join(src_dir, cfg)
        if os.path.isfile(src):
            for dst_dir in dst_dirs:
                shutil.copy2(src, os.path.join(dst_dir, cfg))
                print(f"  Copied {cfg} -> {dst_dir}")


copy_neogps_config()
