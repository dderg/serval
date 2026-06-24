def arc_fit_from_config(config):
    """Read the optional [arc_fit] section, shared by the motion planner and
    the offline viz pipeline so both honor identical knobs and defaults.
    Returns (deviation_tol_mm, min_run_facets), or None when the section is
    absent (arc fitting off)."""
    if not config.has_section("arc_fit"):
        return None
    sc = config.getsection("arc_fit")
    return (
        sc.getfloat("deviation_tol_mm", 0.05, above=0.0),
        sc.getint("min_run_facets", 3, minval=3),
    )
