"""Automated Software-In-the-Loop (STIL) simulator for UAV testing.

A :class:`TestSequence` defines a series of timed control inputs and optional
pass/fail assertion callbacks.  The :class:`STILSimulator` drives the
:class:`~uav_sysid.simulation.flight_dynamics.RigidBodyDynamics` model through
the sequence and records the complete trajectory in a :class:`SimulationResult`.

Example
-------
>>> from uav_sysid.simulation import STILSimulator, TestSequence, UAVState
>>> from uav_sysid.simulation.flight_dynamics import RigidBodyDynamics
>>> dynamics = RigidBodyDynamics(mass=1.5, inertia=[0.02, 0.04, 0.05])
>>> initial_state = UAVState(u=20.0, theta=0.05)
>>> seq = TestSequence(duration=5.0, dt=0.01)
>>> seq.add_step(t_start=0.0, t_end=2.0, controls=[0.0, 0.1, 0.0, 0.5])
>>> seq.add_step(t_start=2.0, t_end=5.0, controls=[0.0, 0.0, 0.0, 0.5])
>>> sim = STILSimulator(dynamics)
>>> result = sim.run(initial_state, seq)
>>> print(result.passed)
True
"""

from __future__ import annotations

import time as _time
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

from .flight_dynamics import RigidBodyDynamics, UAVState


@dataclass
class _ControlStep:
    """A single piecewise-constant control input segment."""

    t_start: float
    t_end: float
    controls: np.ndarray  # shape (M,)
    assertion: Optional[Callable[[UAVState], bool]] = None


class TestSequence:
    """Defines a sequence of timed control inputs for automated STIL testing.

    Parameters
    ----------
    duration : float
        Total simulation duration [s].
    dt : float
        Fixed integration step size [s].
    """

    def __init__(self, duration: float, dt: float = 0.01) -> None:
        if duration <= 0:
            raise ValueError("duration must be positive.")
        if dt <= 0 or dt > duration:
            raise ValueError("dt must be positive and less than duration.")
        self.duration = float(duration)
        self.dt = float(dt)
        self._steps: list[_ControlStep] = []

    # ------------------------------------------------------------------
    # Builder API
    # ------------------------------------------------------------------

    def add_step(
        self,
        t_start: float,
        t_end: float,
        controls,
        assertion: Optional[Callable[[UAVState], bool]] = None,
    ) -> "TestSequence":
        """Add a control step to the sequence.

        Parameters
        ----------
        t_start, t_end : float
            Time interval [s] over which *controls* is applied.
        controls : array-like, shape (M,)
            Control surface deflections / throttle commands.
        assertion : callable(UAVState) -> bool, optional
            If provided, evaluated at *t_end*; the simulation is marked as
            *failed* if the callable returns ``False``.

        Returns
        -------
        TestSequence
            *self* for method chaining.
        """
        if t_start < 0 or t_end > self.duration or t_start >= t_end:
            raise ValueError(
                f"Invalid step interval [{t_start}, {t_end}] "
                f"for sequence of duration {self.duration}."
            )
        self._steps.append(
            _ControlStep(
                t_start=float(t_start),
                t_end=float(t_end),
                controls=np.asarray(controls, dtype=float),
                assertion=assertion,
            )
        )
        return self

    def controls_at(self, t: float) -> np.ndarray:
        """Return the control vector that is active at time *t*.

        The last step whose ``t_start <= t`` is used.  Returns a zero vector
        if no step is active.
        """
        active = np.zeros(4)  # default: 4 control channels
        for step in self._steps:
            if step.t_start <= t:
                active = step.controls.copy()
        return active

    def assertions_at(self, t: float) -> list[Callable[[UAVState], bool]]:
        """Return all assertions whose ``t_end`` equals (or just passed) *t*."""
        return [
            s.assertion
            for s in self._steps
            if s.assertion is not None and abs(s.t_end - t) < self.dt * 0.5
        ]


@dataclass
class SimulationResult:
    """Complete output of a single STIL simulation run.

    Attributes
    ----------
    time : np.ndarray, shape (N,)
        Simulation time vector [s].
    states : list[UAVState]
        UAV state at each time step.
    controls_log : np.ndarray, shape (N, M)
        Control inputs applied at each step.
    passed : bool
        ``True`` if all assertions in the :class:`TestSequence` passed.
    failures : list[str]
        Descriptions of any failed assertions.
    wall_time : float
        Real-time duration of the simulation run [s].
    """

    time: np.ndarray
    states: list[UAVState]
    controls_log: np.ndarray
    passed: bool
    failures: list[str]
    wall_time: float

    # ------------------------------------------------------------------
    # Derived quantities
    # ------------------------------------------------------------------

    def state_array(self) -> np.ndarray:
        """Return all states as an (N, 12) array."""
        return np.stack([s.to_array() for s in self.states])

    def get_channel(self, name: str) -> np.ndarray:
        """Return a named state channel over time.

        Parameters
        ----------
        name : str
            One of ``u, v, w, p, q, r, phi, theta, psi, x, y, z``.
        """
        _idx = dict(
            u=0, v=1, w=2, p=3, q=4, r=5,
            phi=6, theta=7, psi=8, x=9, y=10, z=11,
        )
        if name not in _idx:
            raise KeyError(f"Unknown state channel '{name}'.")
        arr = self.state_array()
        return arr[:, _idx[name]]

    def summary(self) -> str:
        """Return a one-line human-readable summary."""
        status = "PASSED" if self.passed else f"FAILED ({len(self.failures)} failures)"
        return (
            f"SimulationResult: {status} | "
            f"duration={self.time[-1]:.2f}s | steps={len(self.states)} | "
            f"wall_time={self.wall_time:.3f}s"
        )


class STILSimulator:
    """Execute automated Software-In-the-Loop (STIL) simulation runs.

    Parameters
    ----------
    dynamics : RigidBodyDynamics
        The 6-DOF flight dynamics model to use.
    aerodynamics : callable or None
        Optional function ``(state, controls) -> (forces, moments)`` that
        computes aerodynamic forces/moments given the current state and
        control inputs.  When ``None``, only gravity is applied.
    """

    def __init__(
        self,
        dynamics: RigidBodyDynamics,
        aerodynamics: Optional[Callable] = None,
    ) -> None:
        self.dynamics = dynamics
        self.aerodynamics = aerodynamics

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(
        self,
        initial_state: UAVState,
        sequence: TestSequence,
    ) -> SimulationResult:
        """Execute the full test sequence starting from *initial_state*.

        Parameters
        ----------
        initial_state : UAVState
            Initial 12-DOF state.
        sequence : TestSequence
            Timed control inputs and optional assertions.

        Returns
        -------
        SimulationResult
        """
        t_start_wall = _time.monotonic()

        n_steps = int(sequence.duration / sequence.dt)
        time_vec = np.arange(n_steps + 1) * sequence.dt

        states: list[UAVState] = [initial_state]
        ctrl_log_rows: list[np.ndarray] = []
        failures: list[str] = []
        current_state = initial_state

        for i, t in enumerate(time_vec[:-1]):
            controls = sequence.controls_at(t)
            forces, moments = self._compute_forcing(current_state, controls)
            current_state = self.dynamics.step(
                current_state, forces, moments, sequence.dt
            )
            states.append(current_state)
            ctrl_log_rows.append(controls)

            # Evaluate any assertions that are due at the *next* time point
            t_next = time_vec[i + 1]
            for assertion in sequence.assertions_at(t_next):
                if not assertion(current_state):
                    failures.append(
                        f"Assertion failed at t={t_next:.4f}s"
                    )

        ctrl_log_rows.append(ctrl_log_rows[-1] if ctrl_log_rows else np.zeros(4))

        wall_time = _time.monotonic() - t_start_wall
        return SimulationResult(
            time=time_vec,
            states=states,
            controls_log=np.stack(ctrl_log_rows),
            passed=len(failures) == 0,
            failures=failures,
            wall_time=wall_time,
        )

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _compute_forcing(
        self,
        state: UAVState,
        controls: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return (forces, moments) for the current state and controls."""
        if self.aerodynamics is not None:
            return self.aerodynamics(state, controls)
        # Simple gravity-only model
        g = self.dynamics.gravity
        m = self.dynamics.mass
        theta = state.theta
        phi = state.phi
        # Body-frame gravity vector
        Fx = -m * g * np.sin(theta)
        Fy = m * g * np.cos(theta) * np.sin(phi)
        Fz = m * g * np.cos(theta) * np.cos(phi)
        forces = np.array([Fx, Fy, Fz])
        moments = np.zeros(3)
        return forces, moments
