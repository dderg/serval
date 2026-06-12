def load_config(config):
    raise config.error(
        "[firmware_retraction] is not supported: it presupposes an "
        "extruder concept the motion system does not have"
    )
