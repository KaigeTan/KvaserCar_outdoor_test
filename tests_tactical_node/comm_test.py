import time
from typing import Optional

import test_params as parameters
from src.tactical_node.tactical_node.comm_base import  CommInterfaceBase


class CommInterfaceTest(CommInterfaceBase):
    def __init__(self):
        super().__init__()

    def get_latest_message(self) -> Optional[CommInterfaceBase.Message]:
        """
        Always returns the latest available message.
        Non Blocking function
        :return: a message if available, None otherwise
        """
        return CommInterfaceBase.Message(0,
                                         time.perf_counter_ns() // (1000* 100),
                                         (0,5),
                                         (0,5 - parameters.ADV_LENGTH),
                                         2)

    def get_test_message(self, vel, front) -> Optional[CommInterfaceBase.Message]:
        """
        Always returns the latest available message.
        Non Blocking function
        :return: a message if available, None otherwise
        """
        return CommInterfaceBase.Message(0,
                                         time.perf_counter_ns() // (1000* 100),
                                         front,
                                         (0,5 - parameters.ADV_LENGTH),
                                         vel)