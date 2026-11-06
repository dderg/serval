"""PNG of an evaluate() result; needs matplotlib (uv run --group graphs)."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SHAPED_AXES = "xy"
ZOOM_DEVIATIONS = 6.0
MIN_ZOOM_MM = 0.2


def render(result, path):
    s = result["samples"]
    cmd, ex = s["commanded"], s["executed"]
    fig = plt.figure(figsize=(14, 9))
    grid = fig.add_gridspec(2, 2)
    dev = result["executed"]["xy_path_deviation_model"]
    zoom = max(ZOOM_DEVIATIONS * dev["max_mm"], MIN_ZOOM_MM)
    cx, cy = dev["at_xy"]
    for row, limits in ((0, None), (1, (cx, cy, zoom))):
        ax_path = fig.add_subplot(grid[row, 0])
        ax_path.plot(
            cmd["position"]["x"], cmd["position"]["y"], label="commanded"
        )
        ax_path.plot(ex["position"]["x"], ex["position"]["y"], label="executed")
        ax_path.plot(
            s["steps_position"]["x"],
            s["steps_position"]["y"],
            ".",
            markersize=2,
            label="steps",
        )
        ax_path.plot(cx, cy, "rx", label="max dev %.4f mm" % dev["max_mm"])
        if limits:
            ax_path.set_xlim(cx - zoom, cx + zoom)
            ax_path.set_ylim(cy - zoom, cy + zoom)
        ax_path.set_aspect("equal", adjustable="datalim")
        ax_path.set_xlabel("x mm")
        ax_path.set_ylabel("y mm")
        ax_path.legend()
    ax_v = fig.add_subplot(grid[0, 1])
    ax_a = fig.add_subplot(grid[1, 1], sharex=ax_v)
    for axis in SHAPED_AXES:
        ax_v.plot(s["t"], cmd["velocity"][axis], label="cmd v" + axis)
        ax_v.plot(s["t"], ex["velocity"][axis], label="exec v" + axis)
        ax_a.plot(s["t"], cmd["accel"][axis], label="cmd a" + axis)
        ax_a.plot(s["t"], ex["accel"][axis], label="exec a" + axis)
    ax_v.set_ylabel("mm/s")
    ax_a.set_ylabel("mm/s^2")
    ax_a.set_xlabel("print time s")
    ax_v.legend()
    ax_a.legend()
    fig.suptitle("print time %.4f s" % result["print_time_s"])
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
