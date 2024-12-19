"""
GNU Radio APT Encoder module implementation
This module provides tools for encoding APT satellite signals and simulating
Doppler effects with real-time visualization capabilities.
"""
import sys
from gnuradio import gr

# Physical and mathematical constants
EARTH_RADIUS = 6371.0  # Earth radius in km
LIGHT_SPEED = 299792.458  # Speed of light in km/s
NOAA_ALTITUDE = 850.0  # Typical NOAA satellite altitude in km
PI = 3.14159265358979323846
DEG_TO_RAD = PI / 180.0
RAD_TO_DEG = 180.0 / PI

# Initialize flags
_has_svg = False

try:
    from PyQt5.QtSvg import QSvgRenderer
    _has_svg = True
    print("SVG support enabled", file=sys.stderr)
except ImportError:
    print("Warning: SVG support not available. Install PyQt5.QtSvg for map display.", 
          file=sys.stderr)

# Module imports - wrapped in try-except for better error reporting
try:
    from .encoder_python import (
        encoder,
        doppler_simulator,
        doppler_simulator_sgp4,
        AntennaType
    )
except ImportError as e:
    print(f"Error importing encoder_python: {e}", file=sys.stderr)
    raise

try:
    from .position_visualizer import position_visualizer
except ImportError as e:
    print(f"Error importing position_visualizer: {e}", file=sys.stderr)
    raise

try:
    from .apt_encoder_signal_strength_plot import signal_strength_plot
except ImportError as e:
    print(f"Error importing signal_strength_plot: {e}", file=sys.stderr)
    raise

# Version information
__version__ = '1.1.0'

# Public API
__all__ = [
    # Classes
    'encoder',
    'doppler_simulator',
    'doppler_simulator_sgp4',
    'AntennaType',
    'position_visualizer',
    'signal_strength_plot',
    # Constants
    'EARTH_RADIUS',
    'LIGHT_SPEED',
    'NOAA_ALTITUDE',
    'PI',
    'DEG_TO_RAD',
    'RAD_TO_DEG',
    # Internal flags
    '_has_svg'
]

# Module initialization debug output
print(f"APT Encoder module initialized (version {__version__})", file=sys.stderr)
