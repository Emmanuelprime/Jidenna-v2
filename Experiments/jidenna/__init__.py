# robot_control/__init__.py

from .jidenna import Jidenna
from .heading_controller import HeadingHoldController
from .sensor_fusion import SensorFusion
from .serial_comm import SerialComm

__version__ = "0.1.0"
__author__ = "Emmanuel Prime"
__description__ = "Robot control package for Jidenna v2 differential drive robot"

__all__ = [
    'Jidenna',
    'HeadingHoldController',
    'SensorFusion',
    'SerialComm'
]