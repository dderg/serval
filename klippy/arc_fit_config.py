def arc_fit_from_config(config):
    """Read the optional [arc_fit] section, shared by the motion planner and
    the offline viz pipeline so both honor identical knobs and defaults.
    Returns (facet_length_mm, max_angle_deg), or None when the section is
    absent (arc fitting off)."""
    if not config.has_section("arc_fit"):
        return None
    sc = config.getsection("arc_fit")
    return (
        sc.getfloat("facet_length_mm", 1.0, above=0.0),
        sc.getfloat("max_angle_deg", 12.0, above=0.0, below=180.0),
    )
