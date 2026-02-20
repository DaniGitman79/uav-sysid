"""Tests for the STIL simulation module."""

import numpy as np
import pytest

from uav_sysid.simulation import (
    RigidBodyDynamics,
    UAVState,
    STILSimulator,
    SimulationResult,
    TestSequence,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _default_dynamics() -> RigidBodyDynamics:
    return RigidBodyDynamics(mass=1.5, inertia=[0.02, 0.04, 0.05])


def _hover_state() -> UAVState:
    return UAVState(u=20.0, theta=0.05)


# ---------------------------------------------------------------------------
# UAVState tests
# ---------------------------------------------------------------------------

class TestUAVState:
    def test_to_from_array_roundtrip(self):
        s = UAVState(u=20.0, v=1.0, w=0.5, p=0.1, q=0.05, r=0.02,
                     phi=0.1, theta=0.05, psi=1.0, x=100.0, y=50.0, z=-500.0)
        arr = s.to_array()
        s2 = UAVState.from_array(arr)
        assert abs(s2.u - 20.0) < 1e-10
        assert abs(s2.z - (-500.0)) < 1e-10

    def test_airspeed(self):
        s = UAVState(u=20.0, v=0.0, w=0.0)
        assert abs(s.airspeed - 20.0) < 1e-10

    def test_euler_deg(self):
        s = UAVState(phi=np.pi / 6, theta=np.pi / 12, psi=0.0)
        phi_d, theta_d, psi_d = s.euler_deg
        assert abs(phi_d - 30.0) < 1e-6
        assert abs(theta_d - 15.0) < 1e-6


# ---------------------------------------------------------------------------
# RigidBodyDynamics tests
# ---------------------------------------------------------------------------

class TestRigidBodyDynamics:
    def test_invalid_mass(self):
        with pytest.raises(ValueError):
            RigidBodyDynamics(mass=-1.0, inertia=[0.02, 0.04, 0.05])

    def test_invalid_inertia_shape(self):
        with pytest.raises(ValueError):
            RigidBodyDynamics(mass=1.5, inertia=[0.02, 0.04])  # wrong shape

    def test_inertia_3x3(self):
        I = np.diag([0.02, 0.04, 0.05])
        dyn = RigidBodyDynamics(mass=1.5, inertia=I)
        assert dyn.I.shape == (3, 3)

    def test_step_returns_uavstate(self):
        dyn = _default_dynamics()
        s = _hover_state()
        forces = np.array([0.0, 0.0, 0.0])
        moments = np.zeros(3)
        s_next = dyn.step(s, forces, moments, dt=0.01)
        assert isinstance(s_next, UAVState)

    def test_step_gravity_changes_z(self):
        """Without thrust/lift, gravity should accelerate the UAV downward."""
        dyn = _default_dynamics()
        s = UAVState(u=0.0, theta=0.0)   # no initial velocity, wings level
        forces = np.array([0.0, 0.0, dyn.mass * dyn.gravity])  # gravity in body Z
        moments = np.zeros(3)
        s_next = dyn.step(s, forces, moments, dt=0.1)
        # w (downward velocity) should increase
        assert s_next.w > s.w

    def test_derivatives_shape(self):
        dyn = _default_dynamics()
        s = _hover_state()
        deriv = dyn.derivatives(s, np.zeros(3), np.zeros(3))
        assert deriv.shape == (12,)


# ---------------------------------------------------------------------------
# TestSequence tests
# ---------------------------------------------------------------------------

class TestTestSequence:
    def test_invalid_duration(self):
        with pytest.raises(ValueError):
            TestSequence(duration=-1.0)

    def test_invalid_dt(self):
        with pytest.raises(ValueError):
            TestSequence(duration=5.0, dt=10.0)

    def test_invalid_step_interval(self):
        seq = TestSequence(duration=5.0)
        with pytest.raises(ValueError):
            seq.add_step(t_start=3.0, t_end=6.0, controls=[0, 0, 0, 0])

    def test_controls_at(self):
        seq = TestSequence(duration=5.0)
        seq.add_step(0.0, 2.0, controls=[0.1, 0.0, 0.0, 0.5])
        seq.add_step(2.0, 5.0, controls=[0.0, 0.05, 0.0, 0.5])
        ctrl_early = seq.controls_at(1.0)
        ctrl_late = seq.controls_at(3.0)
        assert abs(ctrl_early[0] - 0.1) < 1e-10
        assert abs(ctrl_late[1] - 0.05) < 1e-10

    def test_chaining(self):
        seq = (
            TestSequence(duration=5.0)
            .add_step(0.0, 2.0, [0, 0, 0, 0.5])
            .add_step(2.0, 5.0, [0, 0.1, 0, 0.5])
        )
        assert len(seq._steps) == 2


# ---------------------------------------------------------------------------
# STILSimulator tests
# ---------------------------------------------------------------------------

class TestSTILSimulator:
    def test_run_produces_result(self):
        dyn = _default_dynamics()
        sim = STILSimulator(dyn)
        seq = TestSequence(duration=1.0, dt=0.05)
        seq.add_step(0.0, 1.0, [0.0, 0.0, 0.0, 0.5])
        result = sim.run(_hover_state(), seq)
        assert isinstance(result, SimulationResult)

    def test_result_length(self):
        dyn = _default_dynamics()
        sim = STILSimulator(dyn)
        dt = 0.05
        duration = 1.0
        seq = TestSequence(duration=duration, dt=dt)
        seq.add_step(0.0, 1.0, [0.0, 0.0, 0.0, 0.5])
        result = sim.run(_hover_state(), seq)
        expected_steps = int(duration / dt) + 1
        assert len(result.states) == expected_steps
        assert len(result.time) == expected_steps

    def test_assertion_pass(self):
        dyn = _default_dynamics()
        sim = STILSimulator(dyn)
        seq = TestSequence(duration=1.0, dt=0.1)
        # Assertion that always passes
        seq.add_step(0.0, 1.0, [0, 0, 0, 0.5], assertion=lambda s: True)
        result = sim.run(_hover_state(), seq)
        assert result.passed

    def test_assertion_fail(self):
        dyn = _default_dynamics()
        sim = STILSimulator(dyn)
        seq = TestSequence(duration=1.0, dt=0.1)
        # Assertion that always fails
        seq.add_step(0.0, 1.0, [0, 0, 0, 0.5], assertion=lambda s: False)
        result = sim.run(_hover_state(), seq)
        assert not result.passed
        assert len(result.failures) >= 1

    def test_get_channel(self):
        dyn = _default_dynamics()
        sim = STILSimulator(dyn)
        seq = TestSequence(duration=0.5, dt=0.05)
        seq.add_step(0.0, 0.5, [0.0, 0.0, 0.0, 0.5])
        result = sim.run(_hover_state(), seq)
        u_channel = result.get_channel("u")
        assert u_channel.shape == result.time.shape

    def test_invalid_channel(self):
        dyn = _default_dynamics()
        sim = STILSimulator(dyn)
        seq = TestSequence(duration=0.5, dt=0.05)
        seq.add_step(0.0, 0.5, [0.0, 0.0, 0.0, 0.5])
        result = sim.run(_hover_state(), seq)
        with pytest.raises(KeyError):
            result.get_channel("nonexistent")

    def test_summary_string(self):
        dyn = _default_dynamics()
        sim = STILSimulator(dyn)
        seq = TestSequence(duration=0.5, dt=0.05)
        seq.add_step(0.0, 0.5, [0.0, 0.0, 0.0, 0.5])
        result = sim.run(_hover_state(), seq)
        s = result.summary()
        assert "SimulationResult" in s
        assert "PASSED" in s

    def test_with_aerodynamics_callable(self):
        """Run with a simple aerodynamics model as callable."""
        from uav_sysid.matlab_backend.physics import AerodynamicsModel
        aero = AerodynamicsModel()

        def aero_fn(state, controls):
            return aero.forces_and_moments_from_state(state, controls)

        dyn = _default_dynamics()
        sim = STILSimulator(dyn, aerodynamics=aero_fn)
        seq = TestSequence(duration=0.5, dt=0.05)
        seq.add_step(0.0, 0.5, [0.0, 0.02, 0.0, 0.5])
        result = sim.run(UAVState(u=20.0, theta=0.05), seq)
        assert isinstance(result, SimulationResult)
