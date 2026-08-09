import os
import sys

sys.path.insert(
    0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)

from klippy import mcu_commands, serialhdl  # noqa: E402


class _Command:
    def __init__(self, name, fields):
        self.name = name
        self.param_names = [(field, None) for field in fields]


class _Serial:
    def __init__(self):
        self.calls = []

    def send_args(self, *args):
        self.calls.append(("send_args",) + args)

    def send_with_response_args(self, *args):
        self.calls.append(("send_with_response_args",) + args)
        return {"value": 1}


def _command_wrapper(serial, name="set_output"):
    wrapper = mcu_commands.CommandWrapper.__new__(mcu_commands.CommandWrapper)
    wrapper._serial = serial
    wrapper._cmd = _Command(name, ["oid", "value"])
    return wrapper


def _query_wrapper(serial):
    wrapper = mcu_commands.CommandQueryWrapper.__new__(
        mcu_commands.CommandQueryWrapper
    )
    wrapper._serial = serial
    wrapper._cmd = _Command("query_state", ["oid"])
    wrapper._response = "state"
    wrapper._error = serialhdl.error
    return wrapper


def test_clock_constraints_select_timed_delivery():
    serial = _Serial()
    wrapper = _command_wrapper(serial)
    wrapper.send([3, 4], minclock=100, reqclock=200)
    assert serial.calls == [
        (
            "send_args",
            "set_output",
            [("oid", 3), ("value", 4)],
            serialhdl.CommandDelivery.TIMED,
            100,
            200,
        )
    ]


def test_background_delivery_preserves_minclock():
    serial = _Serial()
    wrapper = _command_wrapper(serial)
    wrapper.send(
        [3, 4],
        minclock=100,
        delivery=serialhdl.CommandDelivery.BACKGROUND,
    )
    assert serial.calls[-1][3:] == (
        serialhdl.CommandDelivery.BACKGROUND,
        100,
        0,
    )


def test_preface_and_query_use_one_ordered_transaction():
    serial = _Serial()
    query = _query_wrapper(serial)
    preface = _command_wrapper(serial, "select_register")
    response = query.send_with_preface(
        preface,
        [7, 8],
        [7],
        minclock=500,
        reqclock=900,
    )
    assert response == {"value": 1}
    assert serial.calls == [
        (
            "send_with_response_args",
            "query_state",
            [("oid", 7)],
            "state",
            serialhdl.CommandDelivery.TIMED,
            500,
            900,
            (
                "select_register",
                [("oid", 7), ("value", 8)],
            ),
        )
    ]


if __name__ == "__main__":
    tests = [
        value for name, value in globals().items() if name.startswith("test_")
    ]
    for test in tests:
        test()
        print("ok", test.__name__)
    print("ALL PASS (%d)" % (len(tests),))
