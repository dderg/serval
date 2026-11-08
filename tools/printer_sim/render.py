"""PNG renders: the outer wall unrolled (side view) and the top surface."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LightSource  # noqa: E402

WALL_BIN_MM = 0.02
DPI = 150


def wall(path, wall_list, layer_height, title):
    """Edge offset d (um) over (position along the outer wall, height):
    ghosting shows as stripes after each corner."""
    layers = sorted({w.layer for w in wall_list})
    length = max(w.s0 + w.s[-1] for w in wall_list) + 1.0
    bins = int(np.ceil(length / WALL_BIN_MM))
    image = np.full((len(layers), bins), np.nan)
    row = {k: i for i, k in enumerate(layers)}
    for w in wall_list:
        grid = np.arange(w.s[0], w.s[-1], WALL_BIN_MM)
        cols = ((w.s0 + grid) / WALL_BIN_MM).astype(int)
        image[row[w.layer], cols] = np.interp(grid, w.s, w.d) * 1e3
    lim = float(np.nanpercentile(np.abs(image), 99.5)) or 1.0
    fig, ax = plt.subplots(figsize=(12, 4))
    im = ax.imshow(
        image,
        origin="lower",
        aspect="auto",
        cmap="RdBu_r",
        vmin=-lim,
        vmax=lim,
        extent=(
            0.0,
            bins * WALL_BIN_MM,
            layers[0] * layer_height,
            (layers[-1] + 1) * layer_height,
        ),
        interpolation="nearest",
    )
    ax.set_xlabel("position along the outer wall (mm)")
    ax.set_ylabel("height (mm)")
    ax.set_title(title)
    fig.colorbar(im, ax=ax, label="edge offset from the intended outline (um)")
    fig.tight_layout()
    fig.savefig(path, dpi=DPI)
    plt.close(fig)


def top(path, layers, nominal_top, title):
    """Shaded surface height relative to the nominal top (layers * h)."""
    grid = layers.grid
    field = layers.top
    covered = field > 0.0
    rows = np.flatnonzero(covered.any(axis=1))
    cols = np.flatnonzero(covered.any(axis=0))
    crop = field[rows[0] : rows[-1] + 1, cols[0] : cols[-1] + 1] - nominal_top
    ls = LightSource(azdeg=315, altdeg=35)
    shaded = ls.shade(
        crop,
        cmap=plt.get_cmap("viridis"),
        vert_exag=20.0,
        blend_mode="overlay",
        dx=grid.dx,
        dy=grid.dx,
    )
    x0 = grid.x0 + cols[0] * grid.dx
    y0 = grid.y0 + rows[0] * grid.dx
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(
        shaded,
        origin="lower",
        extent=(
            x0,
            x0 + crop.shape[1] * grid.dx,
            y0,
            y0 + crop.shape[0] * grid.dx,
        ),
    )
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
