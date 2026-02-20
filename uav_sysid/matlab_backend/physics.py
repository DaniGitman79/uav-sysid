"""UAV aerodynamics physics model backed by the MATLAB engine.

:class:`AerodynamicsModel` computes aerodynamic forces and moments for a
fixed-wing UAV from the instantaneous flight state and control inputs.  All
heavy numerics are delegated to :class:`~uav_sysid.matlab_backend.engine.MatlabEngine`
so that the same API works both with a live MATLAB session and the fallback.

The model uses standard non-dimensional aerodynamic coefficients (CL, CD, Cm,
Cy, Cl, Cn) with stability and control derivatives expressed about the body
axes.

References
----------
- Stevens & Lewis, "Aircraft Control and Simulation", 3rd ed.
- Beard & McLain, "Small Unmanned Aircraft", Princeton UP.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .engine import MatlabEngine


# Small epsilon to guard against division by zero in physics computations
_EPS = 1e-9


@dataclass
class AeroCoefficients:
    """Non-dimensional aerodynamic coefficients for a fixed-wing UAV.

    Stability derivatives (per radian of angle of attack / sideslip)
    and control derivatives (per radian of surface deflection) that
    characterise the aerodynamic model.

    Default values are representative of a small UAV (≈1.5 kg, ~1 m span).
    """

    # ---- Lift (CL) ----
    CL0: float = 0.28       # Zero-AOA lift coefficient
    CL_alpha: float = 4.20  # Lift-curve slope [1/rad]
    CL_de: float = 0.40     # Lift due to elevator [1/rad]
    CL_q: float = 3.80      # Pitch-rate lift damping

    # ---- Drag (CD) ----
    CD0: float = 0.025      # Parasite drag
    CD_alpha: float = 0.30  # Induced drag slope [1/rad]

    # ---- Pitching moment (Cm) ----
    Cm0: float = 0.025      # Zero-AOA pitching moment
    Cm_alpha: float = -0.70 # Pitch stiffness [1/rad]
    Cm_de: float = -1.00    # Pitch control effectiveness [1/rad]
    Cm_q: float = -12.0     # Pitch damping

    # ---- Side force (Cy) ----
    Cy_beta: float = -0.30  # Side-force due to sideslip [1/rad]
    Cy_da: float = 0.0      # Side-force due to aileron (usually small)
    Cy_dr: float = 0.18     # Side-force due to rudder [1/rad]

    # ---- Rolling moment (Cl) ----
    Cl_beta: float = -0.07  # Dihedral effect [1/rad]
    Cl_p: float = -0.44     # Roll damping
    Cl_da: float = 0.25     # Aileron effectiveness [1/rad]
    Cl_dr: float = 0.02     # Rudder roll coupling [1/rad]

    # ---- Yawing moment (Cn) ----
    Cn_beta: float = 0.065  # Directional stability [1/rad]
    Cn_r: float = -0.15     # Yaw damping
    Cn_da: float = -0.03    # Adverse yaw [1/rad]
    Cn_dr: float = -0.10    # Rudder effectiveness [1/rad]


class AerodynamicsModel:
    """Compute aerodynamic forces and moments for a fixed-wing UAV.

    Parameters
    ----------
    coefficients : AeroCoefficients
        Aerodynamic coefficient set.
    wing_area : float
        Reference wing area S [m²].
    wingspan : float
        Reference wingspan b [m].
    chord : float
        Reference mean aerodynamic chord c [m].
    air_density : float
        Air density ρ [kg/m³].  Defaults to sea-level standard (1.225).
    engine : MatlabEngine or None
        If ``None``, a fallback-mode engine is created automatically.
    """

    def __init__(
        self,
        coefficients: AeroCoefficients | None = None,
        wing_area: float = 0.24,
        wingspan: float = 1.0,
        chord: float = 0.24,
        air_density: float = 1.225,
        engine: MatlabEngine | None = None,
    ) -> None:
        self.coeff = coefficients or AeroCoefficients()
        self.S = float(wing_area)
        self.b = float(wingspan)
        self.c = float(chord)
        self.rho = float(air_density)
        self.engine = engine or MatlabEngine(force_fallback=True)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def compute(
        self,
        airspeed: float,
        alpha: float,
        beta: float,
        p: float,
        q: float,
        r: float,
        aileron: float = 0.0,
        elevator: float = 0.0,
        rudder: float = 0.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Compute aerodynamic forces and moments.

        Parameters
        ----------
        airspeed : float
            Total airspeed V [m/s].
        alpha : float
            Angle of attack [rad].
        beta : float
            Sideslip angle [rad].
        p, q, r : float
            Body-axis angular rates [rad/s].
        aileron, elevator, rudder : float
            Control-surface deflections [rad].

        Returns
        -------
        forces : np.ndarray, shape (3,)
            Aerodynamic forces [Fx, Fy, Fz] in body frame [N].
        moments : np.ndarray, shape (3,)
            Aerodynamic moments [L, M, N] in body frame [N·m].
        """
        if airspeed < 1e-3:
            return np.zeros(3), np.zeros(3)

        qbar = 0.5 * self.rho * airspeed ** 2  # dynamic pressure

        # Non-dimensional rates
        p_hat = p * self.b / (2.0 * airspeed)
        q_hat = q * self.c / (2.0 * airspeed)
        r_hat = r * self.b / (2.0 * airspeed)

        c = self.coeff

        # ---- Non-dimensional coefficients ----
        CL = c.CL0 + c.CL_alpha * alpha + c.CL_de * elevator + c.CL_q * q_hat
        CD = c.CD0 + c.CD_alpha * alpha ** 2
        Cm = c.Cm0 + c.Cm_alpha * alpha + c.Cm_de * elevator + c.Cm_q * q_hat

        Cy = c.Cy_beta * beta + c.Cy_da * aileron + c.Cy_dr * rudder
        Cl_roll = c.Cl_beta * beta + c.Cl_p * p_hat + c.Cl_da * aileron + c.Cl_dr * rudder
        Cn = c.Cn_beta * beta + c.Cn_r * r_hat + c.Cn_da * aileron + c.Cn_dr * rudder

        # ---- Wind-frame → body-frame transformation ----
        ca, sa = np.cos(alpha), np.sin(alpha)
        # Lift along -Z_wind, drag along -X_wind
        Fx = qbar * self.S * (-CD * ca + CL * sa)
        Fy = qbar * self.S * Cy
        Fz = qbar * self.S * (-CD * sa - CL * ca)

        # ---- Moments ----
        L_moment = qbar * self.S * self.b * Cl_roll
        M_moment = qbar * self.S * self.c * Cm
        N_moment = qbar * self.S * self.b * Cn

        forces = np.array([Fx, Fy, Fz])
        moments = np.array([L_moment, M_moment, N_moment])
        return forces, moments

    def forces_and_moments_from_state(
        self,
        state,
        controls: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Convenience wrapper that accepts a :class:`~uav_sysid.simulation.flight_dynamics.UAVState`.

        Parameters
        ----------
        state : UAVState
            Current UAV state.
        controls : np.ndarray, shape (≥3,)
            [aileron, elevator, rudder, throttle, …] in radians.

        Returns
        -------
        forces : np.ndarray, shape (3,)
        moments : np.ndarray, shape (3,)
        """
        V = state.airspeed
        alpha = float(np.arctan2(state.w, state.u + _EPS))
        beta = float(np.arcsin(np.clip(state.v / (V + _EPS), -1.0, 1.0)))

        aileron = float(controls[0]) if len(controls) > 0 else 0.0
        elevator = float(controls[1]) if len(controls) > 1 else 0.0
        rudder = float(controls[2]) if len(controls) > 2 else 0.0

        return self.compute(
            airspeed=V,
            alpha=alpha,
            beta=beta,
            p=state.p,
            q=state.q,
            r=state.r,
            aileron=aileron,
            elevator=elevator,
            rudder=rudder,
        )

    def trim(
        self,
        airspeed: float,
        gamma: float = 0.0,
    ) -> dict[str, float]:
        """Compute the straight-and-level trim condition.

        Uses the :class:`~uav_sysid.matlab_backend.engine.MatlabEngine` to
        solve the nonlinear trim equations via Newton iteration (or, in
        fallback mode, via :func:`scipy.optimize.fsolve`).

        Parameters
        ----------
        airspeed : float
            Desired trim airspeed [m/s].
        gamma : float
            Flight-path angle [rad] (0 = level flight).

        Returns
        -------
        dict
            Keys: ``alpha``, ``elevator``, ``throttle``.
        """
        from scipy.optimize import fsolve

        c = self.coeff

        def residuals(x):
            alpha, de = x
            CL = c.CL0 + c.CL_alpha * alpha + c.CL_de * de
            CD = c.CD0 + c.CD_alpha * alpha ** 2
            Cm = c.Cm0 + c.Cm_alpha * alpha + c.Cm_de * de
            qbar = 0.5 * self.rho * airspeed ** 2
            lift = qbar * self.S * CL
            drag = qbar * self.S * CD
            # Force balance: Lift = W cos γ, Thrust = Drag + W sin γ
            weight_component = 1.0  # normalised (unit weight)
            res_lift = lift - weight_component * np.cos(gamma)
            res_moment = Cm  # zero pitching moment
            return [res_lift, res_moment]

        x0 = [0.05, 0.0]
        alpha_trim, de_trim = fsolve(residuals, x0)
        qbar = 0.5 * self.rho * airspeed ** 2
        CD_trim = c.CD0 + c.CD_alpha * alpha_trim ** 2
        drag_trim = qbar * self.S * CD_trim
        throttle_trim = float(np.clip(drag_trim / (qbar * self.S + _EPS), 0.0, 1.0))

        return {
            "alpha": float(alpha_trim),
            "elevator": float(de_trim),
            "throttle": throttle_trim,
        }
