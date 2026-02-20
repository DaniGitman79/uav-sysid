"""UAV system identification via least-squares regression.

The :class:`SystemIdentifier` fits a linear aerodynamic model to recorded
flight data, estimating stability and control derivatives for each of the
three angular-rate channels (roll p, pitch q, yaw r).

Estimated model (per channel, e.g. pitch)
------------------------------------------
::

    q_dot = Mq * q + Mu * u + Mw * w + M_de * de + bias

where ``q`` is the pitch rate, ``u`` and ``w`` are body-axis velocity
components, and ``de`` is elevator deflection.

The same structure is applied to all three channels.

Results are returned as :class:`SysIdResult` dataclass instances.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy import linalg

from .data_loader import FlightData


@dataclass
class SysIdResult:
    """Estimated aerodynamic parameters for one angular-rate channel.

    Attributes
    ----------
    channel : str
        One of ``'roll'``, ``'pitch'``, or ``'yaw'``.
    coefficients : np.ndarray, shape (n_features + 1,)
        Estimated regression coefficients including a bias term.
    feature_names : list[str]
        Names matching each entry in *coefficients* (last entry = ``'bias'``).
    residuals : np.ndarray, shape (N,)
        Prediction residuals on the training data.
    r_squared : float
        Coefficient of determination on the training data.
    """

    channel: str
    coefficients: np.ndarray
    feature_names: list[str]
    residuals: np.ndarray
    r_squared: float

    def as_dict(self) -> dict:
        """Return {feature_name: coefficient} mapping."""
        return dict(zip(self.feature_names, self.coefficients.tolist()))

    def __repr__(self) -> str:
        coef_str = ", ".join(
            f"{n}={v:.4f}" for n, v in zip(self.feature_names, self.coefficients)
        )
        return (
            f"SysIdResult(channel={self.channel!r}, R²={self.r_squared:.4f}, "
            f"coefficients=[{coef_str}])"
        )


class SystemIdentifier:
    """Estimate UAV aerodynamic parameters from flight data.

    The identifier uses ordinary least-squares (via :func:`scipy.linalg.lstsq`)
    with finite-difference angular acceleration as the target signal.

    Parameters
    ----------
    regularisation : float
        L2 regularisation weight (Tikhonov / ridge regression).  Set to
        ``0.0`` to use ordinary least squares without regularisation.
    """

    def __init__(self, regularisation: float = 1e-4) -> None:
        if regularisation < 0:
            raise ValueError("regularisation must be non-negative.")
        self.regularisation = regularisation

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def fit(self, data: FlightData) -> list[SysIdResult]:
        """Fit aerodynamic models to *data*.

        Parameters
        ----------
        data : FlightData
            Pre-processed flight log (see :class:`~uav_sysid.sysid.data_loader.FlightDataLoader`).

        Returns
        -------
        list[SysIdResult]
            One result per angular-rate channel: [roll, pitch, yaw].
        """
        data.validate()

        # Angular accelerations via central finite differences
        omega_dot = self._angular_acceleration(data.gyro, data.dt)
        n = len(omega_dot)

        # Trim all arrays to match the shorter omega_dot length (boundary effect)
        gyro = data.gyro[1:-1]       # shape (n, 3)
        velocity = data.velocity[1:-1]
        controls = data.controls[1:-1]

        results: list[SysIdResult] = []
        channel_names = ["roll", "pitch", "yaw"]
        channel_labels = [
            ["p", "v", "w", *self._ctrl_names(controls, prefix="a")],
            ["q", "u", "w", *self._ctrl_names(controls, prefix="e")],
            ["r", "u", "v", *self._ctrl_names(controls, prefix="r")],
        ]
        # Velocity component indices used per channel
        vel_idx = [
            [1, 2],   # roll  – v, w
            [0, 2],   # pitch – u, w
            [0, 1],   # yaw   – u, v
        ]

        for ch_idx, (ch_name, feat_labels, vi) in enumerate(
            zip(channel_names, channel_labels, vel_idx)
        ):
            y = omega_dot[:, ch_idx]

            # Build regressor matrix: [rate, vel_a, vel_b, controls, 1]
            X_parts = [
                gyro[:, ch_idx : ch_idx + 1],          # own rate
                velocity[:, vi],                         # two velocity components
                controls,                                # all control inputs
                np.ones((n, 1)),                         # bias
            ]
            X = np.hstack(X_parts)
            feature_names = feat_labels + ["bias"]

            coefficients = self._lstsq(X, y)
            y_pred = X @ coefficients
            residuals = y - y_pred
            r2 = self._r_squared(y, y_pred)

            results.append(
                SysIdResult(
                    channel=ch_name,
                    coefficients=coefficients,
                    feature_names=feature_names,
                    residuals=residuals,
                    r_squared=r2,
                )
            )

        return results

    def predict(
        self, result: SysIdResult, data: FlightData
    ) -> np.ndarray:
        """Apply an estimated model to new flight data.

        Parameters
        ----------
        result : SysIdResult
            A result previously produced by :meth:`fit`.
        data : FlightData
            New (or the same) flight data.

        Returns
        -------
        np.ndarray, shape (N,)
            Predicted angular acceleration for the channel.
        """
        channel_map = {"roll": 0, "pitch": 1, "yaw": 2}
        ch_idx = channel_map[result.channel]
        vel_idx_map = {"roll": [1, 2], "pitch": [0, 2], "yaw": [0, 1]}
        vi = vel_idx_map[result.channel]

        n = data.n_samples
        X_parts = [
            data.gyro[:, ch_idx : ch_idx + 1],
            data.velocity[:, vi],
            data.controls,
            np.ones((n, 1)),
        ]
        X = np.hstack(X_parts)
        return X @ result.coefficients

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    @staticmethod
    def _angular_acceleration(
        gyro: np.ndarray, dt: float
    ) -> np.ndarray:
        """Central finite differences of angular rate, shape (N-2, 3)."""
        return (gyro[2:] - gyro[:-2]) / (2.0 * dt)

    def _lstsq(self, X: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Solve the (regularised) least-squares problem."""
        if self.regularisation > 0:
            # Augment with Tikhonov rows: sqrt(lambda) * I
            lam = np.sqrt(self.regularisation)
            n_feat = X.shape[1]
            X_aug = np.vstack([X, lam * np.eye(n_feat)])
            y_aug = np.concatenate([y, np.zeros(n_feat)])
            coef, _, _, _ = linalg.lstsq(X_aug, y_aug)
        else:
            coef, _, _, _ = linalg.lstsq(X, y)
        return coef

    @staticmethod
    def _r_squared(y_true: np.ndarray, y_pred: np.ndarray) -> float:
        ss_res = float(np.sum((y_true - y_pred) ** 2))
        ss_tot = float(np.sum((y_true - np.mean(y_true)) ** 2))
        if ss_tot == 0.0:
            return 1.0 if ss_res == 0.0 else 0.0
        return 1.0 - ss_res / ss_tot

    @staticmethod
    def _ctrl_names(controls: np.ndarray, prefix: str) -> list[str]:
        return [f"{prefix}{i}" for i in range(controls.shape[1])]
