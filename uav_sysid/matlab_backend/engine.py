"""MATLAB engine interface with transparent numpy/scipy fallback.

When the optional ``matlab.engine`` package is installed and a MATLAB licence
is available, :class:`MatlabEngine` delegates computations to a live MATLAB
session.  Otherwise all computations are executed in pure Python using
numpy/scipy, making the rest of the codebase MATLAB-agnostic.

Usage
-----
>>> from uav_sysid.matlab_backend import MatlabEngine
>>> eng = MatlabEngine()          # auto-detects MATLAB or uses fallback
>>> eng.is_matlab_available()
False  # (unless MATLAB is installed in the current environment)
>>> result = eng.eval("1 + 1")    # works in both modes
>>> print(result)
2.0
"""

from __future__ import annotations

from typing import Any, Optional
import numpy as np


class MatlabEngine:
    """Unified interface to MATLAB or the numpy/scipy fallback.

    Parameters
    ----------
    force_fallback : bool
        When ``True``, skip MATLAB detection and always use the Python fallback.
        Useful for testing and CI environments without a MATLAB licence.
    """

    def __init__(self, force_fallback: bool = False) -> None:
        self._matlab_eng: Optional[Any] = None
        self._use_matlab = False

        if not force_fallback:
            self._try_start_matlab()

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def _try_start_matlab(self) -> None:
        """Attempt to start the MATLAB engine; silently fall back on failure."""
        try:
            import matlab.engine  # type: ignore[import]
            self._matlab_eng = matlab.engine.start_matlab()
            self._use_matlab = True
        except Exception:
            # MATLAB not installed or licence not available – use fallback
            self._use_matlab = False

    def is_matlab_available(self) -> bool:
        """Return ``True`` if a live MATLAB session is active."""
        return self._use_matlab

    def close(self) -> None:
        """Shut down the MATLAB engine if it is running."""
        if self._matlab_eng is not None:
            try:
                self._matlab_eng.quit()
            except Exception:
                pass
            self._matlab_eng = None
            self._use_matlab = False

    def __enter__(self) -> "MatlabEngine":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Evaluation
    # ------------------------------------------------------------------

    def eval(self, expression: str) -> Any:
        """Evaluate a MATLAB expression (or equivalent Python expression).

        In MATLAB mode the expression is passed directly to the engine.
        In fallback mode it is evaluated with :func:`eval` in a namespace
        that contains ``numpy`` as ``np``.

        Parameters
        ----------
        expression : str
            MATLAB / Python expression to evaluate.

        Returns
        -------
        Any
            The result of the expression.
        """
        if self._use_matlab and self._matlab_eng is not None:
            return self._matlab_eng.eval(expression, nargout=1)
        # Fallback: evaluate as a Python expression restricted to the numpy
        # namespace.  Only trusted, internally-generated expressions should be
        # passed here; do not expose this method to untrusted user input.
        namespace = {"np": np, "numpy": np, "__builtins__": {}}
        return eval(expression, namespace)  # noqa: S307

    def call(self, func_name: str, *args: Any, nargout: int = 1) -> Any:
        """Call a MATLAB function (or Python/numpy equivalent).

        In MATLAB mode, *func_name* is called directly on the engine.
        In fallback mode, the function is looked up first in ``numpy``, then
        in ``scipy.linalg``, then in Python builtins.

        Parameters
        ----------
        func_name : str
            Name of the MATLAB function (e.g. ``"inv"``, ``"eig"``).
        *args
            Positional arguments forwarded to the function.
        nargout : int
            Number of output arguments (MATLAB mode only).

        Returns
        -------
        Any
        """
        if self._use_matlab and self._matlab_eng is not None:
            matlab_func = getattr(self._matlab_eng, func_name)
            return matlab_func(*args, nargout=nargout)
        return self._fallback_call(func_name, *args)

    # ------------------------------------------------------------------
    # Matrix utilities (mirror common MATLAB matrix functions)
    # ------------------------------------------------------------------

    def inv(self, A) -> np.ndarray:
        """Matrix inverse (``inv`` in MATLAB)."""
        return self.call("inv", np.asarray(A, dtype=float))

    def eig(self, A) -> tuple[np.ndarray, np.ndarray]:
        """Eigenvalues and eigenvectors (``eig`` in MATLAB)."""
        return np.linalg.eig(np.asarray(A, dtype=float))

    def linsolve(self, A, b) -> np.ndarray:
        """Solve linear system A x = b (``linsolve`` in MATLAB)."""
        return np.linalg.solve(
            np.asarray(A, dtype=float), np.asarray(b, dtype=float)
        )

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    @staticmethod
    def _fallback_call(func_name: str, *args: Any) -> Any:
        """Look up *func_name* in numpy / scipy / builtins and call it."""
        import scipy.linalg as _sp_linalg

        _namespaces = [np, _sp_linalg, np.linalg]
        for ns in _namespaces:
            fn = getattr(ns, func_name, None)
            if fn is not None:
                return fn(*args)
        raise AttributeError(
            f"Function '{func_name}' not found in numpy or scipy.linalg. "
            "Connect a live MATLAB engine for full MATLAB function support."
        )
