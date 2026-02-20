"""Tests for the system identification module."""

import numpy as np
import pytest

from uav_sysid.sysid import FlightDataLoader, SystemIdentifier
from uav_sysid.sysid.data_loader import FlightData


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_synthetic_data(n: int = 300, seed: int = 42) -> FlightData:
    """Generate synthetic flight data for testing."""
    rng = np.random.default_rng(seed)
    dt = 0.01
    time = np.arange(n) * dt
    # Simple sinusoidal dynamics
    t = time
    gyro = np.column_stack([
        0.1 * np.sin(2 * np.pi * 0.5 * t),
        0.05 * np.sin(2 * np.pi * 0.3 * t),
        0.02 * np.sin(2 * np.pi * 0.2 * t),
    ])
    velocity = np.column_stack([
        20.0 + rng.normal(0, 0.1, n),
        rng.normal(0, 0.05, n),
        rng.normal(0, 0.05, n),
    ])
    accel = np.column_stack([rng.normal(0, 0.1, n)] * 3)
    euler = np.column_stack([rng.normal(0, 0.05, n)] * 3)
    controls = np.column_stack([
        0.1 * np.sin(2 * np.pi * 0.5 * t),
        0.05 * np.cos(2 * np.pi * 0.3 * t),
        0.02 * np.sin(2 * np.pi * 0.2 * t),
        0.5 * np.ones(n),
    ])
    loader = FlightDataLoader()
    return loader.from_arrays(time, accel, gyro, euler, velocity, controls)


# ---------------------------------------------------------------------------
# FlightDataLoader tests
# ---------------------------------------------------------------------------

class TestFlightDataLoader:
    def test_from_arrays_shape(self):
        data = _make_synthetic_data(100)
        assert data.n_samples == 100
        assert data.accel.shape == (100, 3)
        assert data.gyro.shape == (100, 3)
        assert data.euler.shape == (100, 3)
        assert data.velocity.shape == (100, 3)
        assert data.controls.shape == (100, 4)

    def test_dt(self):
        data = _make_synthetic_data(100)
        assert abs(data.dt - 0.01) < 1e-6

    def test_validation_mismatch(self):
        n = 50
        t = np.arange(n) * 0.01
        zeros3 = np.zeros((n, 3))
        zeros4 = np.zeros((n, 4))
        # Accel has wrong length
        with pytest.raises(ValueError, match="accel"):
            FlightData(
                time=t,
                accel=np.zeros((n + 1, 3)),
                gyro=zeros3,
                euler=zeros3,
                velocity=zeros3,
                controls=zeros4,
            ).validate()

    def test_resample(self):
        data = _make_synthetic_data(100)  # 100 samples at 100 Hz
        loader = FlightDataLoader(resample_hz=50.0)
        resampled = loader.from_arrays(
            data.time, data.accel, data.gyro,
            data.euler, data.velocity, data.controls,
        )
        # 100 samples at 100 Hz → 50 samples at 50 Hz (approx)
        assert resampled.n_samples <= 51
        assert resampled.metadata["resampled_hz"] == 50.0

    def test_from_csv(self, tmp_path):
        """Write a minimal CSV and load it back."""
        import pandas as pd
        n = 20
        t = np.arange(n) * 0.01
        df = pd.DataFrame({
            "time": t,
            "ax": np.zeros(n), "ay": np.zeros(n), "az": np.zeros(n),
            "p": np.zeros(n), "q": np.zeros(n), "r": np.zeros(n),
            "roll": np.zeros(n), "pitch": np.zeros(n), "yaw": np.zeros(n),
            "u": 20.0 * np.ones(n), "v": np.zeros(n), "w": np.zeros(n),
            "aileron": np.zeros(n), "elevator": np.zeros(n),
            "rudder": np.zeros(n), "throttle": 0.5 * np.ones(n),
        })
        csv_path = tmp_path / "flight.csv"
        df.to_csv(csv_path, index=False)
        loader = FlightDataLoader()
        data = loader.from_csv(csv_path)
        assert data.n_samples == n

    def test_from_csv_missing_columns(self, tmp_path):
        import pandas as pd
        df = pd.DataFrame({"time": [0.0, 0.01], "ax": [0.0, 0.0]})
        csv_path = tmp_path / "bad.csv"
        df.to_csv(csv_path, index=False)
        with pytest.raises(ValueError, match="missing required columns"):
            FlightDataLoader().from_csv(csv_path)


# ---------------------------------------------------------------------------
# SystemIdentifier tests
# ---------------------------------------------------------------------------

class TestSystemIdentifier:
    def test_fit_returns_three_results(self):
        data = _make_synthetic_data(300)
        ident = SystemIdentifier()
        results = ident.fit(data)
        assert len(results) == 3
        channels = [r.channel for r in results]
        assert channels == ["roll", "pitch", "yaw"]

    def test_fit_coefficient_shapes(self):
        data = _make_synthetic_data(300)
        ident = SystemIdentifier()
        results = ident.fit(data)
        for r in results:
            # 1 (rate) + 2 (vel) + 4 (controls) + 1 (bias) = 8
            assert len(r.coefficients) == 8
            assert len(r.feature_names) == len(r.coefficients)

    def test_r_squared_range(self):
        data = _make_synthetic_data(300)
        ident = SystemIdentifier()
        results = ident.fit(data)
        for r in results:
            assert 0.0 <= r.r_squared <= 1.0

    def test_as_dict(self):
        data = _make_synthetic_data(300)
        ident = SystemIdentifier()
        result = ident.fit(data)[0]
        d = result.as_dict()
        assert isinstance(d, dict)
        assert len(d) == len(result.coefficients)

    def test_predict_shape(self):
        data = _make_synthetic_data(300)
        ident = SystemIdentifier()
        result = ident.fit(data)[1]  # pitch
        prediction = ident.predict(result, data)
        assert prediction.shape == (data.n_samples,)

    def test_negative_regularisation_raises(self):
        with pytest.raises(ValueError):
            SystemIdentifier(regularisation=-1.0)

    def test_zero_regularisation(self):
        data = _make_synthetic_data(300)
        ident = SystemIdentifier(regularisation=0.0)
        results = ident.fit(data)
        assert len(results) == 3

    def test_repr(self):
        data = _make_synthetic_data(300)
        ident = SystemIdentifier()
        r = ident.fit(data)[0]
        s = repr(r)
        assert "SysIdResult" in s
        assert "roll" in s
