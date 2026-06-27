def arc_fit_from_config(config):
    """Read the optional [arc_fit] section, shared by the motion planner and
    the offline viz pipeline so both honor identical knobs and defaults.
    Returns min_run_facets, or None when the section is absent (arc fitting
    off). The fit tolerance is derived from the machine's square corner
    velocity, not configured."""
    if not config.has_section("arc_fit"):
        return None
    sc = config.getsection("arc_fit")
    return sc.getint("min_run_facets", 3, minval=3)


def arc_fit_heart_from_config(config):
    """Read the optional [arc_fit] heart selector that picks the span-detection
    heart. Returns 'position_greedy' (default) or 'kappa_signal', or None when
    the section or key is absent (the planner then uses its default heart)."""
    if not config.has_section("arc_fit"):
        return None
    sc = config.getsection("arc_fit")
    return sc.getchoice(
        "heart",
        {"position_greedy": "position_greedy", "kappa_signal": "kappa_signal"},
        default="position_greedy",
    )
