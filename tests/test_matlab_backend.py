"""Tests for the MATLAB physics backend module."""

import numpy as np
import pytest

from uav_sysid.matlab_backend import MatlabEngine, AerodynamicsModel, AeroCoefficients
from uav_sysid.simulation.flight_dynamics import UAVState


# ---------------------------------------------------------------------------
# MatlabEngine tests
# ---------------------------------------------------------------------------

class TestMatlabEngine:
    def test_fallback_mode_by_default_when_no_matlab(self):
        eng = MatlabEngine(force_fallback=True)
        assert not eng.is_matlab_available()

    def test_eval_arithmetic(self):
        eng = MatlabEngine(force_fallback=True)
        result = eng.eval("1 + 1")
        assert result == 2

    def test_eval_numpy(self):
        eng = MatlabEngine(force_fallback=True)
        result = eng.eval("np.sqrt(4.0)")
        assert abs(result - 2.0) < 1e-10

    def test_inv(self):
        eng = MatlabEngine(force_fallback=True)
        A = np.array([[2.0, 0.0], [0.0, 4.0]])
        A_inv = eng.inv(A)
        expected = np.diag([0.5, 0.25])
        np.testing.assert_allclose(A_inv, expected, atol=1e-10)

    def test_eig(self):
        eng = MatlabEngine(force_fallback=True)
        A = np.diag([3.0, 5.0])
        vals, _ = eng.eig(A)
        assert set(np.round(vals.real).astype(int).tolist()).issuperset({3, 5})

    def test_linsolve(self):
        eng = MatlabEngine(force_fallback=True)
        A = np.array([[2.0, 0.0], [0.0, 3.0]])
        b = np.array([4.0, 9.0])
        x = eng.linsolve(A, b)
        np.testing.assert_allclose(x, [2.0, 3.0], atol=1e-10)

    def test_call_inv(self):
        eng = MatlabEngine(force_fallback=True)
        A = np.eye(3) * 2.0
        result = eng.call("inv", A)
        np.testing.assert_allclose(result, np.eye(3) * 0.5, atol=1e-10)

    def test_call_unknown_function_raises(self):
        eng = MatlabEngine(force_fallback=True)
        with pytest.raises(AttributeError):
            eng.call("does_not_exist_anywhere", 1.0)

    def test_context_manager(self):
        with MatlabEngine(force_fallback=True) as eng:
            assert not eng.is_matlab_available()
        # After context exit, engine should be closed
        assert not eng.is_matlab_available()


# ---------------------------------------------------------------------------
# AeroCoefficients tests
# ---------------------------------------------------------------------------

class TestAeroCoefficients:
    def test_default_values(self):
        c = AeroCoefficients()
        assert c.CL0 > 0
        assert c.CD0 > 0

    def test_custom_values(self):
        c = AeroCoefficients(CL0=0.5, CD0=0.05)
        assert c.CL0 == 0.5
        assert c.CD0 == 0.05


# ---------------------------------------------------------------------------
# AerodynamicsModel tests
# ---------------------------------------------------------------------------

class TestAerodynamicsModel:
    def test_compute_zero_airspeed(self):
        model = AerodynamicsModel()
        forces, moments = model.compute(
            airspeed=0.0, alpha=0.0, beta=0.0,
            p=0.0, q=0.0, r=0.0,
        )
        np.testing.assert_array_equal(forces, np.zeros(3))
        np.testing.assert_array_equal(moments, np.zeros(3))

    def test_compute_returns_arrays(self):
        model = AerodynamicsModel()
        forces, moments = model.compute(
            airspeed=20.0, alpha=0.05, beta=0.0,
            p=0.0, q=0.0, r=0.0,
            elevator=0.02,
        )
        assert forces.shape == (3,)
        assert moments.shape == (3,)

    def test_lift_positive_alpha(self):
        model = AerodynamicsModel()
        forces_low, _ = model.compute(
            airspeed=20.0, alpha=0.0, beta=0.0,
            p=0.0, q=0.0, r=0.0,
        )
        forces_high, _ = model.compute(
            airspeed=20.0, alpha=0.2, beta=0.0,
            p=0.0, q=0.0, r=0.0,
        )
        # Higher alpha → more lift → more negative Fz (lift opposes gravity in body frame)
        assert forces_high[2] < forces_low[2]

    def test_forces_and_moments_from_state(self):
        model = AerodynamicsModel()
        state = UAVState(u=20.0, v=0.0, w=1.0)
        controls = np.array([0.0, 0.02, 0.0, 0.5])
        forces, moments = model.forces_and_moments_from_state(state, controls)
        assert forces.shape == (3,)
        assert moments.shape == (3,)

    def test_trim_returns_dict(self):
        model = AerodynamicsModel()
        trim = model.trim(airspeed=20.0)
        assert "alpha" in trim
        assert "elevator" in trim
        assert "throttle" in trim

    def test_trim_throttle_range(self):
        model = AerodynamicsModel()
        trim = model.trim(airspeed=20.0)
        assert 0.0 <= trim["throttle"] <= 1.0

    def test_custom_engine(self):
        eng = MatlabEngine(force_fallback=True)
        model = AerodynamicsModel(engine=eng)
        forces, moments = model.compute(
            airspeed=15.0, alpha=0.05, beta=0.0,
            p=0.0, q=0.0, r=0.0,
        )
        assert forces.shape == (3,)
