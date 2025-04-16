import time
from typing import Optional

import tactical_node.parameters as parameters
from tactical_node.comm_base import CommInterfaceBase


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
                                         time.perf_counter_ns() // 1000,
                                         (0,5),
                                         (0,5 - parameters.ADV_LENGTH),
                                         2)