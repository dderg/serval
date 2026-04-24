"""Parity tests for klippy/chelper/jerk_profile.c against the Python
reference at docs/superpowers/plans/plan9-derivations/jerk_profile_ref.py.

Plan 9 Phase A1 — jerk-limited polynomial profile generator.
"""
from __future__ import annotations

import importlib.util
import math
from pathlib import Path

import pytest

from klippy.chelper import jerk_profile as jp


def _load_reference():
    ref_path = (
        Path(__file__).resolve().parents[1]
        / "docs"
        / "superpowers"
        / "plans"
        / "plan9-derivations"
        / "jerk_profile_ref.py"
    )
    spec = importlib.util.spec_from_file_location("jerk_profile_ref", ref_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


REF = _load_reference()


def test_module_importable():
    """Sanity — wrapper and C symbols load cleanly."""
    assert hasattr(jp, "compute_profile")
