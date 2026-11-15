"""Module 4: the bead laid along the simulated nozzle path.

Each nozzle path segment of length L carries the volume the melt passed
in that time. Its bead is A = volume / L in cross-section, flattened to
the layer height h: width w = A / h, capped at max_bead_width where the
nozzle all but stops (a blob, taller than h). The volume is spread evenly
over the grid cells within w/2 of the segment (kernel.c ps_deposit), so a
line's cross-section comes out half-elliptical, 4h/pi high at the centre,
with its edges at +-w/2, and the grid holds exactly the volume deposited.
Layers do not interact: each is laid at its nominal height.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

from . import native

CELL_MM = 0.02
MIN_SEGMENT_CELLS = 0.5
MARGIN_MM = 2.0
COVERED_FRACTION = 0.05
LAYER_Z_TOLERANCE = 0.25


class DepositionError(Exception):
    pass


@dataclasses.dataclass(frozen=True)
class Grid:
    x0: float
    y0: float
    dx: float
    nx: int
    ny: int

    @classmethod
    def around(cls, xy_min, xy_max, dx=CELL_MM, margin=MARGIN_MM):
        x0, y0 = (v - margin for v in xy_min)
        nx = int(math.ceil((xy_max[0] + margin - x0) / dx))
        ny = int(math.ceil((xy_max[1] + margin - y0) / dx))
        return cls(x0, y0, dx, nx, ny)

    def centres(self):
        return (
            self.x0 + (np.arange(self.nx) + 0.5) * self.dx,
            self.y0 + (np.arange(self.ny) + 0.5) * self.dx,
        )

    def empty(self):
        return np.zeros((self.ny, self.nx))


def segments(xy, volume, layer_height, max_width, min_length):
    """Cut the path into pieces at least min_length long: endpoints,
    volume and bead width per piece. volume: cumulative mm^3 at each
    sample."""
    step = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(step)])
    cuts = np.searchsorted(s, np.arange(0.0, s[-1], min_length))
    cuts = np.unique(np.concatenate([cuts, [len(s) - 1]]))
    if len(cuts) < 2:
        return None
    pts = xy[cuts]
    dv = np.diff(volume[cuts])
    length = np.diff(s[cuts])
    with np.errstate(divide="ignore", invalid="ignore"):
        width = np.where(
            length > 0.0,
            dv / (np.maximum(length, 1e-300) * layer_height),
            max_width,
        )
    width = np.clip(width, 0.0, max_width)
    return pts, np.append(dv, 0.0), np.append(width, 0.0)


def deposit(grid: Grid, pts, dv, width):
    ffi, lib = native.load()
    out = grid.empty()
    pts = np.ascontiguousarray(pts, dtype=float)
    x = np.ascontiguousarray(pts[:, 0])
    y = np.ascontiguousarray(pts[:, 1])
    dv = np.ascontiguousarray(dv, dtype=float)
    width = np.ascontiguousarray(width, dtype=float)
    rc = lib.ps_deposit(
        ffi.cast("double *", out.ctypes.data),
        grid.nx,
        grid.ny,
        grid.x0,
        grid.y0,
        grid.dx,
        len(x),
        ffi.cast("double *", x.ctypes.data),
        ffi.cast("double *", y.ctypes.data),
        ffi.cast("double *", dv.ctypes.data),
        ffi.cast("double *", width.ctypes.data),
    )
    if rc:
        raise DepositionError("a bead footprint leaves the grid %r" % (grid,))
    return out


def layer_of(z, layer_height):
    """Layer index of each sample, -1 between layers."""
    k = np.rint(z / layer_height) - 1.0
    on = (
        np.abs(z - (k + 1.0) * layer_height) <= LAYER_Z_TOLERANCE * layer_height
    )
    return np.where(on & (k >= 0), k, -1).astype(int)


@dataclasses.dataclass
class Layers:
    """fields: kept layers' laid height per cell. deposited / flowed: per
    layer, the volume on the grid and the volume the nozzle passed while
    on that layer. top: the surface, in each cell the base of the highest
    layer covering it plus that layer's bead there."""

    grid: Grid
    layer_height: float
    fields: dict
    deposited: dict
    flowed: dict
    top: np.ndarray

    def covered(self, layer):
        return self.fields[layer] > COVERED_FRACTION * self.layer_height


def lay(grid: Grid, xy, z, volume, layer_height, max_width, keep=None):
    """Deposit every layer. volume: cumulative mm^3 out of the nozzle at
    each sample (only increments are laid; suck-back lays nothing). keep:
    layers whose fields are kept (all if None)."""
    layer = layer_of(z, layer_height)
    dv = np.diff(volume, prepend=volume[0])
    positive = np.cumsum(np.maximum(dv, 0.0))
    fields, deposited, flowed = {}, {}, {}
    top = grid.empty()
    for k in np.unique(layer[layer >= 0]):
        idx = np.flatnonzero(layer == k)
        runs = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
        field = grid.empty()
        laid = 0.0
        for run in runs:
            if len(run) < 2:
                continue
            cut = segments(
                xy[run],
                positive[run],
                layer_height,
                max_width,
                MIN_SEGMENT_CELLS * grid.dx,
            )
            if cut is None:
                continue
            pts, seg_dv, width = cut
            field += deposit(grid, pts, seg_dv, width)
            laid += float(seg_dv.sum())
        top = np.where(
            field > COVERED_FRACTION * layer_height,
            k * layer_height + field,
            top,
        )
        deposited[int(k)] = float(field.sum()) * grid.dx**2
        flowed[int(k)] = laid
        if keep is None or int(k) in keep:
            fields[int(k)] = field
    return Layers(grid, layer_height, fields, deposited, flowed, top)
