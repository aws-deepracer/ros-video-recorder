from unittest import TestCase
from unittest.mock import MagicMock
from video_recorder.set_queue import SetQueue


class SetQueueTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_put(self) -> None:
        set_queue = SetQueue()
        set_queue._set = MagicMock()
        set_queue._queue = MagicMock()
        set_queue._cv = MagicMock()
        set_queue._set.__contains__.return_value = False
        set_queue.put("test")
        set_queue._queue.put_nowait.assert_called_once_with("test")
        set_queue._set.add.assert_called_once_with("test")
        set_queue._cv.notify.assert_called_once()

    def test_put_already_in(self) -> None:
        set_queue = SetQueue()
        set_queue._set = MagicMock()
        set_queue._queue = MagicMock()
        set_queue._cv = MagicMock()
        set_queue._set.__contains__.return_value = True
        set_queue.put("test")
        set_queue._queue.put_nowait.assert_not_called()

    def test_get_non_empty_queue(self) -> None:
        set_queue = SetQueue()
        set_queue._set = MagicMock()
        set_queue._queue = MagicMock()
        set_queue._cv = MagicMock()
        set_queue._queue.empty.return_value = False
        get_value = set_queue.get()
        set_queue._queue.get_nowait.assert_called_once_with()
        set_queue._set.remove.assert_called_once_with(get_value)

    def test_get_empty_queue_and_notify(self) -> None:
        set_queue = SetQueue()
        set_queue._set = MagicMock()
        set_queue._queue = MagicMock()
        set_queue._cv = MagicMock()
        set_queue._queue.empty.return_value = True
        set_queue._cv.wait.return_value = True
        get_value = set_queue.get()
        set_queue._queue.get_nowait.assert_called_once_with()
        set_queue._set.remove.assert_called_once_with(get_value)

    def test_get_empty_queue_and_not_notify(self) -> None:
        set_queue = SetQueue()
        set_queue._set = MagicMock()
        set_queue._queue = MagicMock()
        set_queue._cv = MagicMock()
        set_queue._queue.empty.return_value = True
        set_queue._cv.wait.return_value = False
        with self.assertRaises(Exception):
            set_queue.get()

    def test_empty(self) -> None:
        set_queue = SetQueue()
        set_queue._set = set()
        set_queue._queue = MagicMock()
        set_queue._queue.empty.return_value = True
        set_queue._cv = MagicMock()
        self.assertTrue(set_queue.empty())

    def test_non_empty(self) -> None:
        set_queue = SetQueue()
        set_queue._set = set()
        set_queue._queue = MagicMock()
        set_queue._queue.empty.return_value = False
        set_queue._cv = MagicMock()
        self.assertFalse(set_queue.empty())

    def test_contains(self) -> None:
        set_queue = SetQueue()
        set_queue._set = set(["test1"])
        self.assertTrue("test1" in set_queue)

    def test_not_contains(self) -> None:
        set_queue = SetQueue()
        set_queue._set = set([])
        self.assertFalse("test1" in set_queue)
