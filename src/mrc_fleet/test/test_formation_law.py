"""Tests of the follower control law (pure math, no ROS)."""
import math

from mrc_fleet.formation_controller import compose, lookahead_cmd

L, K, VMAX, WMAX = 0.2, 0.8, 0.26, 1.5
OFF = (-0.8, 0.7)


def slot(lead):
    x, y = compose(*lead, *OFF)
    return (x, y, lead[2])


def test_zero_command_when_parked_and_leader_stopped():
    lead = (1.0, 2.0, 0.7)
    v, w = lookahead_cmd(lead, 0.0, 0.0, *OFF, slot(lead), L, K, VMAX, WMAX)
    assert abs(v) < 1e-9 and abs(w) < 1e-9


def test_pure_feedforward_on_straight_line():
    lead = (0.0, 0.0, 0.3)
    v, w = lookahead_cmd(lead, 0.15, 0.0, *OFF, slot(lead), L, K, VMAX, WMAX)
    assert abs(v - 0.15) < 1e-9 and abs(w) < 1e-9


def test_rigid_turn_feedforward():
    # leader turning left: the right-hand (outer) follower must go faster
    lead = (0.0, 0.0, 0.0)
    vr, _ = lookahead_cmd(lead, 0.1, 0.1, -0.8, -0.7, slot_r := (*compose(*lead, -0.8, -0.7), 0.0),
                          L, K, VMAX, WMAX)
    vl, _ = lookahead_cmd(lead, 0.1, 0.1, *OFF, slot(lead), L, K, VMAX, WMAX)
    assert slot_r and vr > 0.1 > vl


def simulate(lead_v, lead_w, start_err, steps=600, dt=0.05):
    lead = [0.0, 0.0, 0.0]
    sx, sy, sth = slot(tuple(lead))
    me = [sx + start_err[0], sy + start_err[1], sth + start_err[2]]
    for _ in range(steps):
        v, w = lookahead_cmd(tuple(lead), lead_v, lead_w, *OFF, tuple(me), L, K, VMAX, WMAX)
        me[0] += v * math.cos(me[2]) * dt
        me[1] += v * math.sin(me[2]) * dt
        me[2] += w * dt
        lead[0] += lead_v * math.cos(lead[2]) * dt
        lead[1] += lead_v * math.sin(lead[2]) * dt
        lead[2] += lead_w * dt
    s = slot(tuple(lead))
    return math.hypot(s[0] - me[0], s[1] - me[1])


def test_converges_on_straight_line_from_offset():
    assert simulate(0.11, 0.0, (-0.3, 0.25, 0.6)) < 0.02


def test_converges_with_leader_crawling():
    # leader throttled to ~10 % (waiting for followers): the old Kanayama law could
    # not remove a lateral error here because its correction scaled with leader speed
    assert simulate(0.022, 0.0, (0.0, 0.2, 0.0), steps=1600) < 0.03


def test_tracks_during_turn():
    # U-turn of the default route: leader 0.11 m/s on radius 0.9 m (inner slot)
    # inner slot barely moves forward (0.025 m/s): small residual from the static blend
    assert simulate(0.11, 0.11 / 0.9, (0.1, -0.1, 0.2), steps=500) < 0.05


def test_outer_slot_during_turn():
    global OFF
    old, OFF = OFF, (-0.8, -0.7)
    try:
        assert simulate(0.10, 0.10 / 0.9, (0.0, 0.0, 0.0), steps=500) < 0.03
    finally:
        OFF = old
