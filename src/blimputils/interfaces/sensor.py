"""
Abstract sensor interface.
"""

from abc import ABC, abstractmethod
from typing import List

class SensorInterface(ABC):
    """
    Abstract base class for a generic 3-axis sensor.
    """

    @abstractmethod
    def get_x(self) -> int:
        """
        Get the raw X-axis sensor value.

        :return: The raw X-axis value.
        :rtype: int
        """
        ...

    @abstractmethod
    def get_y(self) -> int:
        """
        Get the raw Y-axis sensor value.

        :return: The raw Y-axis value.
        :rtype: int
        """
        ...

    @abstractmethod
    def get_z(self) -> int:
        """
        Get the raw Z-axis sensor value.

        :return: The raw Z-axis value.
        :rtype: int
        """
        ...

    @abstractmethod
    def get_xyz(self) -> List[int]:
        """
        Get the raw X, Y, and Z-axis sensor values.

        :return: A list containing [X, Y, Z] sensor values.
        :rtype: list[int]
        """
        ...
