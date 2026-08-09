import klippy.configfile


def configwrapper_to_dict(
    wrapper: klippy.configfile.ConfigWrapper,
) -> dict[str, dict[str, str]]:
    fileconfig = wrapper.fileconfig
    return {
        section: {
            option: fileconfig.get(section, option)
            for option in fileconfig.options(section)
        }
        for section in fileconfig.sections()
    }
