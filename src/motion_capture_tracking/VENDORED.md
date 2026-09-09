# Vendored: motion_capture_tracking (do NOT install the apt package)

Source: https://github.com/IMRCLab/motion_capture_tracking, branch `ros2`,
commit `64d3af2` (2026-07-21), with `deps/libmotioncapture` at `24321e4`
(2026-07-17, "Revert 'vibecoded troubleshooting from chatgpt'").
Nested `.git` metadata was stripped; this is a plain copy.

## Why it is vendored

- The apt release `ros-humble-motion-capture-tracking 1.0.9` (and the 1.0.9
  build queued for Jazzy) pins a libmotioncapture commit that hard-codes a
  TU Berlin interface address, `141.23.110.162`, into the NatNet data-socket
  `bind()` / `join_group()`. On any other machine the node either aborts
  (exit -6, "set_option: No such device") or silently never binds and
  publishes an empty `/poses` forever. Upstream reverted it on 2026-07-17 but
  no fixed release exists.
- Every apt version (1.0.6 and 1.0.9) mis-parses the NatNet 4.2 model
  definition sent by Motive 3.4.1 and can segfault after the handshake,
  depending on which rigid bodies Motive streams.

Building from this tree gives Ubuntu 22.04 / Humble and 24.04 / Jazzy the same
working parser. `scripts/install_deps.sh` removes the apt package so it cannot
shadow this build.

## Local patch applied on top of upstream

`patches/0003-libmotioncapture-natnet-4.2-modeldef-segfault.patch`
(copied into this directory) — for NatNet >= 4.1 use each dataset's
`description_size` as the authoritative boundary in `parseModelDef`, read only
rigid-body name / ID / parent / offsets, skip the rest.
Applies with:

    git apply --directory=src/motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture \
        src/motion_capture_tracking/patches/0003-libmotioncapture-natnet-4.2-modeldef-segfault.patch

## Updating

Re-clone upstream at a newer commit, re-apply the patch, verify
`grep -r 141.23.110.162 motion_capture_tracking/deps/libmotioncapture/src`
prints nothing, strip `.git` entries, rebuild, and confirm `/poses` streams.
