"""6-DOF rigid-body flight dynamics for UAV STIL simulation.

The equations of motion follow standard aerospace conventions:
- Body-fixed axes (NED: forward-right-down)
- Euler angles (roll φ, pitch θ, yaw ψ)
- Forces in Newtons, moments in N·m

Reference: Stevens & Lewis, "Aircraft Control and Simulation", 3rd ed.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.integrate import solve_ivp


# Numerical guard constants
_LARGE_TAN = 1e6   # Replaces tan(θ) near gimbal-lock (|θ| → 90°)
_EPS = 1e-9        # Small value to prevent division by zero


@dataclass
class UAVState:
    """Full 12-state UAV rigid-body state vector.

    Attributes
    ----------
    u, v, w : float
        Body-frame velocity components [m/s].
    p, q, r : float
        Body-frame angular rates [rad/s].
    phi, theta, psi : float
        Euler roll, pitch, yaw angles [rad].
    x, y, z : float
        Position in NED inertial frame [m].
    """

    u: float = 0.0
    v: float = 0.0
    w: float = 0.0
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0
    phi: float = 0.0
    theta: float = 0.0
    psi: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def to_array(self) -> np.ndarray:
        """Return state as a 12-element array [u,v,w, p,q,r, φ,θ,ψ, x,y,z]."""
        return np.array(
            [
                self.u, self.v, self.w,
                self.p, self.q, self.r,
                self.phi, self.theta, self.psi,
                self.x, self.y, self.z,
            ]
        )

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "UAVState":
        """Construct a :class:`UAVState` from a 12-element array."""
        arr = np.asarray(arr, dtype=float)
        return cls(
            u=arr[0], v=arr[1], w=arr[2],
            p=arr[3], q=arr[4], r=arr[5],
            phi=arr[6], theta=arr[7], psi=arr[8],
            x=arr[9], y=arr[10], z=arr[11],
        )

    @property
    def airspeed(self) -> float:
        """Total airspeed [m/s] (assuming no wind)."""
        return float(np.sqrt(self.u**2 + self.v**2 + self.w**2))

    @property
    def euler_deg(self) -> tuple[float, float, float]:
        """Roll, pitch, yaw in degrees."""
        return (
            float(np.degrees(self.phi)),
            float(np.degrees(self.theta)),
            float(np.degrees(self.psi)),
        )


class RigidBodyDynamics:
    """6-DOF rigid-body equations of motion for a fixed-wing UAV.

    Parameters
    ----------
    mass : float
        Aircraft mass [kg].
    inertia : array-like, shape (3, 3) or (3,)
        Inertia tensor [kg·m²].  A 1-D input of length 3 is interpreted as
        the diagonal [Ixx, Iyy, Izz] with zero off-diagonal terms.
    gravity : float
        Gravitational acceleration [m/s²].  Defaults to 9.81.
    """

    def __init__(
        self,
        mass: float,
        inertia,
        gravity: float = 9.81,
    ) -> None:
        if mass <= 0:
            raise ValueError("mass must be positive.")
        self.mass = float(mass)
        self.gravity = float(gravity)

        inertia = np.asarray(inertia, dtype=float)
        if inertia.ndim == 1 and len(inertia) == 3:
            self.I = np.diag(inertia)
        elif inertia.shape == (3, 3):
            self.I = inertia.copy()
        else:
            raise ValueError("inertia must be shape (3,) or (3, 3).")

        self.I_inv = np.linalg.inv(self.I)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def step(
        self,
        state: UAVState,
        forces: np.ndarray,
        moments: np.ndarray,
        dt: float,
    ) -> UAVState:
        """Integrate the equations of motion for one time step.

        Parameters
        ----------
        state : UAVState
            Current state.
        forces : np.ndarray, shape (3,)
            External forces [Fx, Fy, Fz] in body frame [N].
        moments : np.ndarray, shape (3,)
            External moments [L, M, N] in body frame [N·m].
        dt : float
            Integration step size [s].

        Returns
        -------
        UAVState
            Updated state after *dt* seconds.
        """
        x0 = state.to_array()
        forces = np.asarray(forces, dtype=float)
        moments = np.asarray(moments, dtype=float)

        sol = solve_ivp(
            fun=lambda _t, x: self._derivatives(x, forces, moments),
            t_span=(0.0, dt),
            y0=x0,
            method="RK45",
            max_step=dt,
        )
        return UAVState.from_array(sol.y[:, -1])

    def derivatives(
        self,
        state: UAVState,
        forces: np.ndarray,
        moments: np.ndarray,
    ) -> np.ndarray:
        """Return the time derivatives of the state vector (for inspection)."""
        return self._derivatives(state.to_array(), forces, moments)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _derivatives(
        self,
        x: np.ndarray,
        forces: np.ndarray,
        moments: np.ndarray,
    ) -> np.ndarray:
        """Compute ẋ given state *x*, external *forces* and *moments*."""
        u, v, w = x[0], x[1], x[2]
        p, q, r = x[3], x[4], x[5]
        phi, theta = x[6], x[7]

        Fx, Fy, Fz = forces
        L, M, N = moments

        g = self.gravity
        m = self.mass

        # Translational dynamics (body frame)
        u_dot = Fx / m - g * np.sin(theta) + r * v - q * w
        v_dot = Fy / m + g * np.cos(theta) * np.sin(phi) - r * u + p * w
        w_dot = Fz / m + g * np.cos(theta) * np.cos(phi) + q * u - p * v

        # Rotational dynamics
        omega = np.array([p, q, r])
        gyroscopic = np.cross(omega, self.I @ omega)
        omega_dot = self.I_inv @ (np.array([L, M, N]) - gyroscopic)
        p_dot, q_dot, r_dot = omega_dot

        # Euler angle kinematics
        cos_phi = np.cos(phi)
        sin_phi = np.sin(phi)
        cos_theta = np.cos(theta)
        tan_theta = np.tan(theta) if abs(cos_theta) > 1e-6 else np.sign(cos_theta) * _LARGE_TAN

        phi_dot = p + (q * sin_phi + r * cos_phi) * tan_theta
        theta_dot = q * cos_phi - r * sin_phi
        psi_dot = (q * sin_phi + r * cos_phi) / (cos_theta + _EPS)

        # Position kinematics (body → inertial)
        psi = x[8]
        sin_theta = np.sin(theta)
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)

        # Rotation matrix body → NED
        R = np.array(
            [
                [
                    cos_theta * cos_psi,
                    sin_phi * sin_theta * cos_psi - cos_phi * sin_psi,
                    cos_phi * sin_theta * cos_psi + sin_phi * sin_psi,
                ],
                [
                    cos_theta * sin_psi,
                    sin_phi * sin_theta * sin_psi + cos_phi * cos_psi,
                    cos_phi * sin_theta * sin_psi - sin_phi * cos_psi,
                ],
                [
                    -sin_theta,
                    sin_phi * cos_theta,
                    cos_phi * cos_theta,
                ],
            ]
        )
        vel_ned = R @ np.array([u, v, w])
        x_dot, y_dot, z_dot = vel_ned

        return np.array(
            [
                u_dot, v_dot, w_dot,
                p_dot, q_dot, r_dot,
                phi_dot, theta_dot, psi_dot,
                x_dot, y_dot, z_dot,
            ]
        )
