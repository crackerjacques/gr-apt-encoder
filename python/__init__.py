"""
GNU Radio APT Encoder module implementation
This module provides tools for encoding APT satellite signals and simulating
Doppler effects with real-time visualization capabilities.
"""
from gnuradio import gr
from PyQt5.QtSvg import QSvgRenderer

# Physical and mathematical constants
EARTH_RADIUS = 6371.0  # Earth radius in km
LIGHT_SPEED = 299792.458  # Speed of light in km/s
NOAA_ALTITUDE = 850.0  # Typical NOAA satellite altitude in km
PI = 3.14159265358979323846
DEG_TO_RAD = PI / 180.0
RAD_TO_DEG = 180.0 / PI

# Module imports
from .encoder_python import encoder, doppler_simulator, AntennaType
from .position_visualizer import position_visualizer
from .apt_encoder_signal_strength_plot import signal_strength_plot

__version__ = '1.1.0'

# Check for SVG support
try:
    _has_svg = True
    print("SVG support enabled")
except ImportError:
    _has_svg = False
    print("Warning: SVG support not available. Install PyQt5.QtSvg for map display.")

__all__ = [
    # Classes
    'encoder',
    'doppler_simulator',
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