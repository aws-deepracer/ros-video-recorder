#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################
"""A class for Set Queue"""
import queue

from threading import Condition
from typing import Any


class SetQueue():
    """
    SetQueue class

    Because a set is use in SetQueue, so it does not allow duplicated items
    """

    def __init__(self) -> None:
        """
        SetQueue constructor
        """
        super().__init__()
        self._cv = Condition()
        self._queue = queue.Queue()
        self._set = set()

    def put(self, item: Any) -> None:
        """
        Put an item into the queue.

        Args:
            item (Any): item to enqueue
        """
        with self._cv:
            if item not in self._set:
                self._queue.put_nowait(item)
                self._set.add(item)
                self._cv.notify()

    def get(self, timeout: float = None) -> Any:
        """
        Get an item from the queue with wait timeout.

        Args:
            timeout (float): timeout in seconds before an exception will be throw.

        Returns:
            Any: item from the queue.
        """
        with self._cv:
            # for timetout, the return value is True
            # unless a given timeout expired, in which case it is False
            if self._queue.empty() and not self._cv.wait(timeout=timeout):
                raise queue.Empty
            item = self._queue.get_nowait()
            self._set.remove(item)
            return item

    def empty(self) -> bool:
        """
        Return whether SetQueue is empty or not.

        Returns:
            bool: Return True if the queue is empty, False otherwise
        """
        with self._cv:
            return self._queue.empty()

    def __contains__(self, item: Any) -> bool:
        """
        Return whether item is in SetQueue in runtime O(1)

        Args:
            item (Any): item to check in SetQueue

        Returns:
            bool: True if item in SetQueue and False otherwise
        """
        with self._cv:
            return item in self._set
