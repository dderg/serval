# Utility for querying the current state of all endstops
#
# Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class QueryEndstops:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.endstops = []
        self.last_state = []
        # Register webhook if server is available
        webhooks = self.printer.lookup_object("webhooks")
        webhooks.register_endpoint(
            "query_endstops/status", self._handle_web_request
        )
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "QUERY_ENDSTOPS",
            self.cmd_QUERY_ENDSTOPS,
            desc=self.cmd_QUERY_ENDSTOPS_help,
        )
        gcode.register_command("M119", self.cmd_QUERY_ENDSTOPS)

    def register_endstop(self, mcu_endstop, name):
        self.endstops.append((mcu_endstop, name))

    def get_status(self, eventtime):
        return {"last_query": {name: value for name, value in self.last_state}}

    def _handle_web_request(self, web_request):
        gc_mutex = self.printer.lookup_object("gcode").get_mutex()
        toolhead = self.printer.lookup_object("toolhead")
        with gc_mutex:
            print_time = toolhead.get_last_move_time()
            self.last_state = [
                (name, _query_state(mcu_endstop, print_time))
                for mcu_endstop, name in self.endstops
            ]
        web_request.send({name: _state_name(t) for name, t in self.last_state})

    cmd_QUERY_ENDSTOPS_help = "Report on the status of each endstop"

    def cmd_QUERY_ENDSTOPS(self, gcmd):
        # Query the endstops
        print_time = self.printer.lookup_object("toolhead").get_last_move_time()
        self.last_state = [
            (name, _query_state(mcu_endstop, print_time))
            for mcu_endstop, name in self.endstops
        ]
        # Report results
        msg = " ".join(
            ["%s:%s" % (name, _state_name(t)) for name, t in self.last_state]
        )
        gcmd.respond_raw(msg)


# An endstop that cannot be sampled like a pin (an EtherCAT servo rail's)
# names its state through query_endstop_state(); its query_endstop() is
# for homing and raises.
def _query_state(mcu_endstop, print_time):
    query_state = getattr(mcu_endstop, "query_endstop_state", None)
    if query_state is not None:
        return query_state(print_time)
    return mcu_endstop.query_endstop(print_time)


def _state_name(state):
    if isinstance(state, str):
        return state
    return ["open", "TRIGGERED"][not not state]


def load_config(config):
    return QueryEndstops(config)
