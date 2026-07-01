# Anti-tearing: why "golden64"

## The problem

This is a 240×320 ST7789 controller driven **landscape** (`MADCTL 0x60`, the MV=1 transpose). That
transpose makes the panel's own hardware refresh sweep **perpendicular** to the direction the firmware
writes scanlines. With no tearing-effect (TE) sync line wired, and not enough RAM for a second
framebuffer, the write raster and the scan raster run at slightly different, unsynchronized rates — so
the boundary between the newly-written frame and the previous one shows up as a **diagonal tear seam**.

After working the problem thoroughly we established that on this hardware the seam **cannot be dissolved**
at a playable 60 fps (dissolving it would need either TE sync or a full back-buffer, neither of which
exists here). That leaves two things you *can* do, both at a locked 60 fps:

## Option A — STABLE: park one clean line

Write scanlines top-to-bottom in order and use the frame pacer to match the panel's free-run refresh, so
the single sharp diagonal seam **holds still** in one place instead of drifting.

- **Look:** crisp, grain-free image with one stationary diagonal line — clean, but the line *is* visible,
  most of all when content scrolls across it.
- Cheaper; less RAM.

## Option B — golden64: scatter the seam into noise  *(chosen)*

Write each 64-row window in a scrambled **golden-ratio order** (`perm[i] = (i · STEP) mod N`, with
`N = 64` bands and `STEP ≈ N / φ`) so successive rows land as far apart as possible. The new/old boundary
is then no longer a single sharp edge but a **faint ~1px-tall noise ribbon** along the diagonal.

- **Look:** no clean line — instead a subtle grain/shimmer where the seam was. The eye objects to *lines*
  far more than to *scattered noise*, so in motion this reads as "less of a tear."
- Trade-off: a slightly grainier texture in flat areas, and more RAM (a deeper write-ahead ring, ~52% of
  the 256 KB). Same locked 60 fps as STABLE.

## Decision (2026-06-30)

**golden64** is the shipped default — the faint scattered grain is preferred over a single clean parked
line. STABLE remains available as a rollback if the crisp-line look is ever wanted instead.

> Note: the [square 240×240 sibling](https://github.com/superslor/pico-infonesPlus-240) is too slow at
> per-line window commands for scatter to hold 60 fps, so it uses the STABLE parked-line approach instead.
