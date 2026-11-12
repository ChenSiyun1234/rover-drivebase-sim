"""Protection, fixture and suspension checks independent of saved outcomes."""
import numpy as np
import pytest
import mujoco
from sim.motor import ProtectedMotor
from sim.power import Battery
from sim.drive import TorqueGovernor, WHEELS
from sim.can_dm import CanBus, encode_velocity, encode_feedback, decode_feedback
from sim.rocker_bogie import solve_loads, predict_loads
from experiments.run_all import Case, load_scene


@pytest.mark.parametrize('fault', [9, 10, 13])
def test_fault_latches_and_delayed_feedback(fault):
    motor = ProtectedMotor(behaviour='trip', timeout=.25)
    motor.command(20 if fault == 10 else 2, 0)
    assert motor.check(0, 14 if fault == 9 else 24, .3 if fault == 13 else .1) == 0
    assert motor.error == fault
    motor.command(0, 1)
    assert motor.check(0, 24, 1) == 0
    bus = CanBus(motor_ids=[1])
    bus.send(encode_velocity(1, 0), 1)
    bus.advance(1.001, {1: (0, 0, 0, motor.error)})
    bus.advance(1.002, {1: (0, 0, 0, motor.error)})
    assert bus.feedback[1]['error'] == fault
    motor.enable(2)
    motor.command(2, 2)
    assert motor.check(0, 24, 2) == 2


def test_current_limited_pack_matches_constant_power_root():
    pack = Battery()
    v, current, torque = pack.solve([100]*6, [0]*6, current_limit=8)
    power = 6*8**2*2.25
    root = (25.2+np.sqrt(25.2**2-4*.04*power))/2
    assert v == pytest.approx(root, abs=1e-6)
    assert current == pytest.approx(power/v, abs=1e-6)
    assert torque == pytest.approx([8]*6)


def test_hand_computed_suspension_equilibrium():
    a = np.array([[1,1,1,1,1,1], [1,1,1,-1,-1,-1],
                  [1,0,-1,1,0,-1], [0,.5,-.5,0,0,0],
                  [0,0,0,0,.5,-.5], [1,0,-1,-1,0,1]], float)
    assert solve_loads(a, [-300,0,0,0,0,0]) == pytest.approx([50]*6)


def test_loaded_scene_static_prediction():
    model = load_scene(Case())
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    points = np.array([data.xpos[model.body('wheel_'+w).id] for w in WHEELS])
    points[:, 2] -= .12
    loads, residual = predict_loads(model, data, points, np.zeros((6,3)))
    assert loads.sum() == pytest.approx(50*9.81)
    assert loads[:3] == pytest.approx(loads[3:])
    assert np.all(loads > 0)
    assert np.max(np.abs(residual)) < 1e-10


def test_fixtures_keep_mass_and_only_change_intended_environment():
    baseline = load_scene(Case())
    obstacle = load_scene(Case(obstacle=True))
    locked = load_scene(Case(stall=True))
    assert baseline.body_mass.sum() == obstacle.body_mass.sum() == locked.body_mass.sum() == 50
    assert obstacle.ngeom == baseline.ngeom+1
    assert locked.neq == baseline.neq+6
    assert locked.nv == baseline.nv


def feedback(torque, error=1):
    return {i: decode_feedback(encode_feedback(i, 0, 0, torque, error)) for i in range(1,7)}


def test_governor_reduces_recovers_and_inhibits_faults():
    governor = TorqueGovernor()
    for _ in range(100):
        governor.update(.5, feedback(7.8))
    reduced = governor.scale
    assert 0 < reduced < .8
    assert governor.update(.5, feedback(6.4)) == pytest.approx(.5*reduced)
    for _ in range(100):
        governor.update(.5, feedback(2))
    assert governor.scale == pytest.approx(reduced+.03)
    assert governor.update(.5, feedback(0, 10)) == 0
    assert TorqueGovernor().update(.5, {}) == 0
