#!/usr/bin/env python3
"""
Relocate PX4's external-modules block in the root CMakeLists.txt so it is
configured AFTER platforms/ and src/lib (and the in-tree modules) are added.

Why: every module links against `px4_platform` (line ~169 of px4_add_module.cmake),
which is what carries the px4_platform_common include directories. PX4 configures
external modules BEFORE platforms/ exists, so that target isn't defined yet, CMake
degrades it to a bare link flag, the includes are dropped, and the module compiles
without ModuleBase visible ("expected template-name before '<'"). Moving the block
after the targets exist fixes milestone 1 and also lets milestone 2 use
`DEPENDS px4_work_queue drivers__device`.

Content-matched (not line-number based) and idempotent: safe to re-run, and to
re-apply after bumping PX4 to a new tag. Usage:

    python3 patch_px4_external_order.py /path/to/PX4-Autopilot/CMakeLists.txt
"""
import sys

ANCHOR = "add events lib after modules"          # insert the block just before this
SET_LINE = "set(external_module_paths)"          # top of the block
IF_LINE = "if (NOT EXTERNAL_MODULES_LOCATION"    # block guard
INTREE_LOOP = "foreach(module ${config_module_list})"  # marks in-tree module loop


def main(path):
    with open(path) as f:
        lines = f.readlines()

    # locate the block
    try:
        i_set = next(i for i, l in enumerate(lines) if l.strip() == SET_LINE)
    except StopIteration:
        sys.exit("could not find 'set(external_module_paths)' — unexpected CMake layout")

    # include contiguous comment/separator lines directly above the set()
    start = i_set
    while start > 0 and lines[start - 1].lstrip().startswith("#"):
        start -= 1

    # find the endif() that closes the block's if()
    if IF_LINE not in lines[i_set + 1] and not any(IF_LINE in l for l in lines[i_set:i_set + 3]):
        sys.exit("block guard 'if (NOT EXTERNAL_MODULES_LOCATION' not where expected")
    end = next(i for i in range(i_set, len(lines)) if lines[i].strip().startswith("endif("))

    block = lines[start:end + 1]

    i_anchor = next((i for i, l in enumerate(lines) if ANCHOR in l), None)
    if i_anchor is None:
        sys.exit("could not find insertion anchor ('%s')" % ANCHOR)

    i_loop = next((i for i, l in enumerate(lines) if INTREE_LOOP in l), -1)

    # idempotency: already moved if the block sits after the in-tree module loop
    if start > i_loop >= 0:
        print("already patched — external block is after the in-tree module loop; no change")
        return

    # remove the block, then insert before the anchor (recompute anchor after removal)
    del lines[start:end + 1]
    i_anchor = next(i for i, l in enumerate(lines) if ANCHOR in l)
    lines[i_anchor:i_anchor] = block + ["\n"]

    with open(path, "w") as f:
        f.writelines(lines)
    print("patched: moved external-modules block to just before '%s'" % ANCHOR)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
