"""Flight data loader for UAV system identification.

Supports loading and pre-processing flight-log data from CSV files or
in-memory NumPy arrays.  The resulting :class:`FlightData` object is the
standard input consumed by :class:`~uav_sysid.sysid.estimator.SystemIdentifier`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd


@dataclass
class FlightData:
    """Container for a single UAV flight log.

    All arrays must share the same first dimension (number of time samples).

    Attributes
    ----------
    time : np.ndarray, shape (N,)
        Elapsed time in seconds.
    accel : np.ndarray, shape (N, 3)
        Body-frame accelerations [ax, ay, az] in m/s².
    gyro : np.ndarray, shape (N, 3)
        Body-frame angular rates [p, q, r] in rad/s.
    euler : np.ndarray, shape (N, 3)
        Euler angles [roll, pitch, yaw] in radians.
    velocity : np.ndarray, shape (N, 3)
        Body-frame velocities [u, v, w] in m/s.
    controls : np.ndarray, shape (N, M)
        Control surface deflections (aileron, elevator, rudder, throttle, …)
        in radians or normalised units.
    """

    time: np.ndarray
    accel: np.ndarray
    gyro: np.ndarray
    euler: np.ndarray
    velocity: np.ndarray
    controls: np.ndarray
    metadata: dict = field(default_factory=dict)

    # ------------------------------------------------------------------
    # Derived quantities
    # ------------------------------------------------------------------

    @property
    def n_samples(self) -> int:
        """Number of time samples."""
        return len(self.time)

    @property
    def dt(self) -> float:
        """Mean sample interval in seconds."""
        if self.n_samples < 2:
            return 0.0
        return float(np.mean(np.diff(self.time)))

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------

    def validate(self) -> None:
        """Raise *ValueError* if any array has an inconsistent length."""
        n = self.n_samples
        for name in ("accel", "gyro", "euler", "velocity", "controls"):
            arr = getattr(self, name)
            if arr.shape[0] != n:
                raise ValueError(
                    f"Array '{name}' has {arr.shape[0]} rows but 'time' has {n}."
                )


class FlightDataLoader:
    """Load and pre-process UAV flight logs.

    Parameters
    ----------
    resample_hz : float or None
        If given, the data is resampled to this frequency via linear
        interpolation after loading.  Set to ``None`` to skip resampling.
    """

    # Expected CSV column groups
    _TIME_COL = "time"
    _ACCEL_COLS = ["ax", "ay", "az"]
    _GYRO_COLS = ["p", "q", "r"]
    _EULER_COLS = ["roll", "pitch", "yaw"]
    _VEL_COLS = ["u", "v", "w"]
    _CTRL_COLS = ["aileron", "elevator", "rudder", "throttle"]

    def __init__(self, resample_hz: Optional[float] = None) -> None:
        self.resample_hz = resample_hz

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def from_csv(self, path: str | Path) -> FlightData:
        """Load a flight log from a CSV file.

        The CSV must contain at minimum a ``time`` column plus the columns
        listed in :attr:`_ACCEL_COLS`, :attr:`_GYRO_COLS`,
        :attr:`_EULER_COLS`, :attr:`_VEL_COLS`, and :attr:`_CTRL_COLS`.

        Parameters
        ----------
        path : str or Path
            Path to the CSV file.

        Returns
        -------
        FlightData
        """
        df = pd.read_csv(path)
        return self._df_to_flight_data(df, metadata={"source": str(path)})

    def from_arrays(
        self,
        time: np.ndarray,
        accel: np.ndarray,
        gyro: np.ndarray,
        euler: np.ndarray,
        velocity: np.ndarray,
        controls: np.ndarray,
    ) -> FlightData:
        """Create a :class:`FlightData` directly from NumPy arrays.

        Parameters
        ----------
        time : np.ndarray, shape (N,)
        accel : np.ndarray, shape (N, 3)
        gyro : np.ndarray, shape (N, 3)
        euler : np.ndarray, shape (N, 3)
        velocity : np.ndarray, shape (N, 3)
        controls : np.ndarray, shape (N, M)

        Returns
        -------
        FlightData
        """
        data = FlightData(
            time=np.asarray(time, dtype=float),
            accel=np.asarray(accel, dtype=float),
            gyro=np.asarray(gyro, dtype=float),
            euler=np.asarray(euler, dtype=float),
            velocity=np.asarray(velocity, dtype=float),
            controls=np.asarray(controls, dtype=float),
        )
        data.validate()
        if self.resample_hz is not None:
            data = self._resample(data, self.resample_hz)
        return data

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _df_to_flight_data(self, df: pd.DataFrame, metadata: dict) -> FlightData:
        """Convert a DataFrame to :class:`FlightData`."""
        missing = [
            c
            for c in [self._TIME_COL]
            + self._ACCEL_COLS
            + self._GYRO_COLS
            + self._EULER_COLS
            + self._VEL_COLS
            + self._CTRL_COLS
            if c not in df.columns
        ]
        if missing:
            raise ValueError(f"CSV is missing required columns: {missing}")

        def _arr(cols: list[str]) -> np.ndarray:
            return df[cols].to_numpy(dtype=float)

        ctrl_cols = [c for c in self._CTRL_COLS if c in df.columns]

        data = FlightData(
            time=df[self._TIME_COL].to_numpy(dtype=float),
            accel=_arr(self._ACCEL_COLS),
            gyro=_arr(self._GYRO_COLS),
            euler=_arr(self._EULER_COLS),
            velocity=_arr(self._VEL_COLS),
            controls=_arr(ctrl_cols),
            metadata=metadata,
        )
        data.validate()
        if self.resample_hz is not None:
            data = self._resample(data, self.resample_hz)
        return data

    @staticmethod
    def _resample(data: FlightData, target_hz: float) -> FlightData:
        """Linearly interpolate *data* to a uniform *target_hz* grid."""
        t_start = data.time[0]
        t_end = data.time[-1]
        n_new = int((t_end - t_start) * target_hz) + 1
        t_new = np.linspace(t_start, t_end, n_new)

        def _interp(arr: np.ndarray) -> np.ndarray:
            if arr.ndim == 1:
                return np.interp(t_new, data.time, arr)
            return np.column_stack(
                [np.interp(t_new, data.time, arr[:, i]) for i in range(arr.shape[1])]
            )

        return FlightData(
            time=t_new,
            accel=_interp(data.accel),
            gyro=_interp(data.gyro),
            euler=_interp(data.euler),
            velocity=_interp(data.velocity),
            controls=_interp(data.controls),
            metadata={**data.metadata, "resampled_hz": target_hz},
        )
