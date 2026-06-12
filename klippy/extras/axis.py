RESERVED_LETTERS = ("i", "j", "p", "q", "f", "g", "m", "n", "t")


class AxisSection:
    def __init__(self, config):
        self.name = config.get_name().split(None, 1)[1]
        if (
            len(self.name) != 1
            or not self.name.islower()
            or not self.name.isalpha()
        ):
            raise config.error(
                "[%s]: axis name must be a single letter a-z"
                % config.get_name()
            )
        if self.name in RESERVED_LETTERS:
            raise config.error(
                "[%s]: letter '%s' is reserved for G-code structure"
                % (config.get_name(), self.name)
            )
        self.follows = [
            a.strip().lower() for a in config.getlist("follows", [])
        ]
        self.motors = [m.strip() for m in config.getlist("motors", [])]

    def get_status(self, eventtime):
        return {"follows": list(self.follows), "motors": list(self.motors)}


def load_config_prefix(config):
    return AxisSection(config)
