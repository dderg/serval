import pytest


def test_module_imports():
    import motion_engine

    assert hasattr(motion_engine, "MotionEngine")


def test_engine_instantiates():
    import motion_engine

    engine = motion_engine.MotionEngine()
    assert engine.version() != ""


def test_claim_mcu_returns_int():
    import motion_engine

    engine = motion_engine.MotionEngine()
    handle = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    assert isinstance(handle, int)


def test_claim_two_mcus_returns_distinct_handles():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h1 = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    h2 = engine.claim_mcu("mcu2", "/dev/ttyACM1", 250000)
    assert h1 != h2


def test_release_mcu_then_alloc_fails():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    engine.release_mcu(h)
    with pytest.raises(RuntimeError):
        engine.alloc_command_queue(h)


def test_alloc_command_queue():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    q = engine.alloc_command_queue(h)
    assert isinstance(q, int)


def test_alloc_two_queues_distinct():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    q1 = engine.alloc_command_queue(h)
    q2 = engine.alloc_command_queue(h)
    assert q1 != q2


def test_passthrough_send_does_not_crash():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    q = engine.alloc_command_queue(h)
    engine.passthrough_send(h, q, b"\x01\x02\x03")


def test_passthrough_send_with_clocks():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    q = engine.alloc_command_queue(h)
    engine.passthrough_send(h, q, b"\xaa", min_clock=100, req_clock=200)


def test_passthrough_query_returns_notify_id():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    q = engine.alloc_command_queue(h)
    nid = engine.passthrough_query(h, q, b"\x01")
    assert isinstance(nid, int)
    assert nid > 0


def test_send_wait_ack_raises_not_implemented():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    q = engine.alloc_command_queue(h)
    with pytest.raises(NotImplementedError, match="Phase 2"):
        engine.passthrough_send_wait_ack(h, q, b"\x01", 1.0)


def test_register_handler_does_not_crash():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    engine.passthrough_register_handler(h, "get_status", 0, lambda params: None)


def test_register_flush_callback_does_not_crash():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    engine.passthrough_register_flush_callback(h, lambda: None)


def test_poll_event_returns_none_when_empty():
    import motion_engine

    engine = motion_engine.MotionEngine()
    assert engine.poll_event() is None


def test_add_config_cmd_and_begin_config_phase():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    added = engine.add_config_cmd(h, b"\x10\x20")
    assert added is True
    engine.begin_config_phase(h)
    added_after = engine.add_config_cmd(h, b"\x30\x40")
    assert added_after is False


def test_add_init_cmd():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    added = engine.add_init_cmd(h, b"\xaa")
    assert added is True


def test_add_restart_cmd():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    added = engine.add_restart_cmd(h, b"\xbb")
    assert added is True


def test_get_stats_returns_dict():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    stats = engine.get_stats(h)
    assert isinstance(stats, dict)
    assert stats["bytes_write"] == 0
    assert stats["send_seq"] == 0
    assert "ready_bytes" in stats


def test_set_clock_est():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    engine.set_clock_est(h, 48_000_000.0, 0.0, 1000)


def test_next_config_entry_after_config_phase():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    engine.add_config_cmd(h, b"\x01")
    engine.add_config_cmd(h, b"\x02")
    engine.begin_config_phase(h)
    e1 = engine.next_config_entry(h)
    assert e1 is not None
    e2 = engine.next_config_entry(h)
    assert e2 is not None
    e3 = engine.next_config_entry(h)


def test_extract_old_returns_dict():
    import motion_engine

    engine = motion_engine.MotionEngine()
    h = engine.claim_mcu("mcu", "/dev/ttyACM0", 250000)
    result = engine.extract_old(h)
    assert isinstance(result, dict)
    assert "sent" in result
    assert "received" in result
    assert isinstance(result["sent"], list)
    assert isinstance(result["received"], list)


def test_unknown_mcu_raises_runtime_error():
    import motion_engine

    engine = motion_engine.MotionEngine()
    with pytest.raises(RuntimeError, match="unknown MCU"):
        engine.alloc_command_queue(999)


def test_unknown_mcu_get_stats_raises():
    import motion_engine

    engine = motion_engine.MotionEngine()
    with pytest.raises(RuntimeError, match="unknown MCU"):
        engine.get_stats(999)
