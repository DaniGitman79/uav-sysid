"""MATLAB Physics Backend sub-package.

Provides a MATLAB-compatible interface for UAV aerodynamic and physics
computations.  When a live MATLAB engine is available it is used; otherwise
the pure-Python (numpy/scipy) fallback is used transparently.
"""

from .engine import MatlabEngine
from .physics import AerodynamicsModel, AeroCoefficients

__all__ = ["MatlabEngine", "AerodynamicsModel", "AeroCoefficients"]
