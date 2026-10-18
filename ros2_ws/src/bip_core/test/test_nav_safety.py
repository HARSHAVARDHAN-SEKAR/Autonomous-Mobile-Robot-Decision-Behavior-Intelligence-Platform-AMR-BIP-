"""Regression tests for wall-adjacent heading correction and block gating."""

from bip_core.nav_safety import command_for_heading


def cmd(error, blocked=False, allow_turn=True):
    return command_for_heading(error, distance_remaining=2.0,
                               max_lin=0.35, max_ang=1.2,
                               front_blocked=blocked, allow_turn=allow_turn)


def test_blocked_turn_has_zero_forward_speed():
    linear, angular = cmd(-0.5, blocked=True)
    assert linear == 0.0 and angular < 0.0


def test_blocked_straight_cannot_drive_into_wall():
    assert cmd(0.1, blocked=True) == (0.0, 0.0)


def test_blocked_unknown_turn_clearance_is_not_a_free_pass():
    assert cmd(-1.0, blocked=True, allow_turn=False) == (0.0, 0.0)


def test_large_heading_error_rotates_before_advancing():
    linear, angular = cmd(-0.5)
    assert linear == 0.0 and angular < 0.0


def test_aligned_unblocked_drives_forward():
    linear, angular = cmd(0.0)
    assert 0.0 < linear <= 0.35 and angular == 0.0


def test_rotation_speed_clamped():
    _, angular = cmd(3.0)
    assert angular == 1.2


def test_turn_threshold_continuity_for_forward_motion():
    linear, angular = cmd(0.34)
    assert 0.0 < linear < 0.35 and angular > 0.0
