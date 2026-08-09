class PostProcessorSection:
    def __init__(self, config):
        self.name = config.get_name().split(None, 1)[1]
        self.type = config.get("type")
        self.params = [
            (opt, config.getfloat(opt))
            for opt in config.get_prefix_options("")
            if opt != "type"
        ]

    def get_status(self, eventtime):
        return {"type": self.type, "params": dict(self.params)}


def load_config_prefix(config):
    return PostProcessorSection(config)
