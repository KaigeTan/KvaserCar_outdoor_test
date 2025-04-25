
from typing import Tuple, Optional

class ComMsg:
    def __init__(self, id, time_stamp, front, back, vel, length=None):
        """

        :param time_stamp:
        :param front:
        :param back:
        :param vel:
        """
        self.id = id
        self.time_stamp: int = time_stamp
        self.front: Tuple[float, float] = front
        self.back: Tuple[float, float] = back
        self.velocity: float = vel
        self.length: Optional[float] = length