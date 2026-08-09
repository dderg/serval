def load_config(config):
    raise config.error(
        "[gcode_arcs] is temporarily not supported: the motion engine has no "
        "native G2/G3 arc ingestion yet. Remove the [gcode_arcs] section; emit "
        "G1 segments from the slicer until native arc support lands"
    )
