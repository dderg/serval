import json
import os

from . import servo_axis

KIN_COREXY = 0
KIN_CARTESIAN = 1


class BeltPair:
    def __init__(self, rail, node, kin_tag, lane_a, lane_b):
        self.rail = rail
        self.node = node
        self.kin_tag = kin_tag
        self.lane_a = lane_a
        self.lane_b = lane_b
        self.motors = servo_axis.rail_motors_in_slot_order(rail)

    def axis_name(self):
        return self.rail.get_name(short=True)

    def motor_names(self):
        return [motor.get_motor_name() for motor in self.motors]

    def slots(self):
        return [
            self.node.get_slot_for_motor(motor.get_motor_name())
            for motor in self.motors
        ]

    def mech_signs(self):
        return [
            -1.0 if motor.get_invert_direction() else 1.0
            for motor in self.motors
        ]


class ServoStrainComp:
    cmd_SERVO_STRAIN_COMP_help = (
        "ENABLE=1 uploads the map file to the endpoint (offsets ramp in at "
        "1 mm/s); ENABLE=0 clears the compensation."
    )

    def __init__(self, config):
        self.printer = config.get_printer()
        self.map_file = os.path.expanduser(
            config.get("map_file", "~/printer_data/config/strain_comp.json")
        )
        self._measured_stiffness = {}
        self._measured_cross = {}
        self.printer.lookup_object("gcode").register_command(
            "SERVO_STRAIN_COMP",
            self.cmd_SERVO_STRAIN_COMP,
            desc=self.cmd_SERVO_STRAIN_COMP_help,
        )

    def enumerate_belt_pairs(self, gcmd, axis_filter=None):
        toolhead = self.printer.lookup_object("toolhead")
        kin = toolhead.get_kinematics()
        lanes = [
            (lane_idx, kin.rails[lane_idx])
            for lane_idx, _axis_name, _motors in kin.lanes()
            if isinstance(kin.rails[lane_idx], servo_axis.ServoRail)
            and len(kin.rails[lane_idx].get_motors()) == 2
            and kin.rails[lane_idx].get_name(short=True) != "z"
        ]
        if len(lanes) != 2:
            raise gcmd.error(
                "strain compensation needs exactly two dual-drive belt axes, "
                "found %d" % len(lanes)
            )
        kin_tag = KIN_COREXY if kin.coupled_xy() else KIN_CARTESIAN
        lane_a, lane_b = lanes[0][0], lanes[1][0]
        pairs = []
        for lane_idx, rail in lanes:
            if (
                axis_filter is not None
                and rail.get_name(short=True) != axis_filter
            ):
                continue
            node = self.printer.lookup_object(
                "ethercat_node " + rail.get_node_name()
            )
            pairs.append(BeltPair(rail, node, kin_tag, lane_a, lane_b))
        if not pairs:
            raise gcmd.error(
                "no belt pair matching AXIS=%s" % axis_filter.upper()
            )
        return pairs

    def get_engine(self):
        return self.printer.lookup_object("motion_engine")

    def get_engine_handle(self, gcmd, pair):
        handle = pair.node.get_engine_handle()
        if handle is None:
            raise gcmd.error(
                "ethercat_node %s has no engine handle" % pair.node.name
            )
        return handle

    def apply_pair_constant(self, gcmd, pair, value_um):
        self._set_pair(gcmd, pair, 1, 1, 0.0, 0.0, 1.0, 1.0, [int(value_um)])

    def apply_pair_grid(self, gcmd, pair, grid):
        self._set_pair(
            gcmd,
            pair,
            grid["nx"],
            grid["ny"],
            grid["x0"],
            grid["y0"],
            grid["dx"],
            grid["dy"],
            [int(value) for value in grid["offsets_um"]],
        )

    def clear_pair_grid(self, gcmd, pair):
        self._set_pair(gcmd, pair, 0, 0, 0.0, 0.0, 1.0, 1.0, [])

    def _set_pair(self, gcmd, pair, nx, ny, x0, y0, dx, dy, offsets):
        self.get_engine().set_strain_comp(
            self.get_engine_handle(gcmd, pair),
            pair.slots()[0],
            pair.slots()[1],
            pair.lane_a,
            pair.lane_b,
            pair.kin_tag,
            nx,
            ny,
            x0,
            y0,
            dx,
            dy,
            offsets,
        )

    def measured_stiffness(self):
        return self._measured_stiffness

    def measured_cross(self):
        return self._measured_cross

    def cmd_SERVO_STRAIN_COMP(self, gcmd):
        if gcmd.get_int("ENABLE", 1, minval=0, maxval=1) == 0:
            for pair in self.enumerate_belt_pairs(gcmd):
                self.clear_pair_grid(gcmd, pair)
            gcmd.respond_info("strain compensation cleared (ramping out)")
            return
        self.enable_from_map(gcmd)

    def enable_from_map(self, gcmd, quiet=False):
        if not os.path.exists(self.map_file):
            raise gcmd.error(
                "no map file at %s — record one with SERVO_MEASURE_STRAIN_MAP "
                "and build it with SERVO_STRAIN_COMP_BUILD" % self.map_file
            )
        with open(self.map_file) as fh:
            payload = json.load(fh)
        by_motors = {
            tuple(entry["motors"]): entry for entry in payload["pairs"]
        }
        for pair in self.enumerate_belt_pairs(gcmd):
            key = tuple(pair.motor_names())
            entry = by_motors.pop(key, None)
            if entry is None:
                raise gcmd.error(
                    "map file has no entry for belt %s (%s) — rebuild it against "
                    "this printer's motors" % (pair.axis_name(), "/".join(key))
                )
            self.apply_pair_grid(gcmd, pair, entry)
            if not quiet:
                gcmd.respond_info(
                    "belt %s compensation enabled: %dx%d grid, offsets %+d..%+d um"
                    % (
                        pair.axis_name(),
                        entry["nx"],
                        entry["ny"],
                        min(entry["offsets_um"]),
                        max(entry["offsets_um"]),
                    )
                )
        if by_motors:
            raise gcmd.error(
                "map file entries %s match no belt pair on this printer"
                % ", ".join("/".join(key) for key in by_motors)
            )


def load_config(config):
    return ServoStrainComp(config)
