#!/usr/bin/env python3
"""
Apply the PX4-tree patches this out-of-tree module needs, against a given
PX4-Autopilot checkout. Both patches are content-matched (not line-number based)
and idempotent, so this is safe to re-run and to re-apply after bumping PX4 to a
new tag. Keep it in your build script.

  1. Root CMakeLists.txt: move the external-modules block to after the in-tree
     module loop, so platforms/ and src/lib targets exist when the external
     module is configured (fixes include env + lets milestone-2 DEPENDS resolve).

  2. src/modules/logger/logged_topics.cpp: add sensor_flow_angle to
     add_default_topics() so the custom topic lands in the onboard ulog alongside
     the standard set (appends; does not disturb the defaults).

Usage:
    python3 patch_px4.py /path/to/PX4-Autopilot
"""
import os
import sys

# ---- patch 1: external-modules reorder in root CMakeLists.txt ----
ANCHOR = "add events lib after modules"
SET_LINE = "set(external_module_paths)"
IF_LINE = "if (NOT EXTERNAL_MODULES_LOCATION"
INTREE_LOOP = "foreach(module ${config_module_list})"


def patch_cmake(path):
    with open(path) as f:
        lines = f.readlines()

    try:
        i_set = next(i for i, l in enumerate(lines) if l.strip() == SET_LINE)
    except StopIteration:
        print("  cmake: 'set(external_module_paths)' not found — skipping")
        return

    start = i_set
    while start > 0 and lines[start - 1].lstrip().startswith("#"):
        start -= 1

    if not any(IF_LINE in l for l in lines[i_set:i_set + 3]):
        print("  cmake: block guard not where expected — skipping")
        return
    end = next(i for i in range(i_set, len(lines)) if lines[i].strip().startswith("endif("))
    block = lines[start:end + 1]

    i_anchor = next((i for i, l in enumerate(lines) if ANCHOR in l), None)
    if i_anchor is None:
        print("  cmake: insertion anchor not found — skipping")
        return
    i_loop = next((i for i, l in enumerate(lines) if INTREE_LOOP in l), -1)

    if start > i_loop >= 0:
        print("  cmake: already patched (external block after in-tree loop)")
        return

    del lines[start:end + 1]
    i_anchor = next(i for i, l in enumerate(lines) if ANCHOR in l)
    lines[i_anchor:i_anchor] = block + ["\n"]
    with open(path, "w") as f:
        f.writelines(lines)
    print("  cmake: moved external-modules block after the in-tree module loop")


# ---- patch 2: log sensor_flow_angle in logged_topics.cpp ----
FUNC = "void LoggedTopics::add_default_topics()"
LOG_LINE = '\tadd_topic("sensor_flow_angle", 100);\n'


def patch_logger(path):
    with open(path) as f:
        lines = f.readlines()

    if any("sensor_flow_angle" in l for l in lines):
        print("  logger: already patched (sensor_flow_angle present)")
        return

    i_func = next((i for i, l in enumerate(lines) if FUNC in l), None)
    if i_func is None:
        print("  logger: add_default_topics() not found — skipping")
        return
    # find the opening brace line at/after the function signature
    i_brace = next(i for i in range(i_func, len(lines)) if lines[i].strip() == "{")
    lines[i_brace + 1:i_brace + 1] = [LOG_LINE]
    with open(path, "w") as f:
        f.writelines(lines)
    print("  logger: added sensor_flow_angle to add_default_topics()")


def main(root):
    cmake = os.path.join(root, "CMakeLists.txt")
    logger = os.path.join(root, "src", "modules", "logger", "logged_topics.cpp")
    for label, p in (("cmake", cmake), ("logger", logger)):
        if not os.path.isfile(p):
            sys.exit("not found: %s (is %s a PX4-Autopilot checkout?)" % (p, root))
    print("patching %s" % root)
    patch_cmake(cmake)
    patch_logger(logger)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
