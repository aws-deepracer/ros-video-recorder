from unittest import TestCase
from unittest.mock import MagicMock
from video_recorder.constants import VideoRecorderState
from video_recorder.video_recorder import VideoRecorder


class VideoRecorderTest(TestCase):
    def setUp(self) -> None:
        self.video_name = "test"
        self.video_topic = "test"
        self.writer = MagicMock()
        self.uploader = MagicMock()
        self.subscriber = MagicMock()

    def test_state(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._state = VideoRecorderState.RUN
        self.assertTrue(video_recorder.state == VideoRecorderState.RUN)

    def test_video_name(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        self.assertTrue(video_recorder.video_name == self.video_name)

    def test_is_frame_queue_emtpy(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder.is_empty()
        video_recorder._subscriber.is_empty.assert_called_once()

    def test_exceed_max_frames(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder.exceed_max_frames()
        video_recorder._writer.exceed_max_frames.assert_called_once()

    def test_should_immediate_stop_true(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._writer.should_immediate_stop = True
        self.assertTrue(video_recorder.should_immediate_stop())

    def test_should_immediate_stop_false(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._writer.should_immediate_stop = False
        self.assertFalse(video_recorder.should_immediate_stop())

    def test_is_terminated_true(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._writer.is_terminated.return_value = True
        video_recorder._uploader.is_terminated.return_value = True
        self.assertTrue(video_recorder.is_terminated())

    def test_is_terminated_false_test1(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._writer.is_terminated.return_value = True
        video_recorder._uploader.is_terminated.return_value = False
        self.assertFalse(video_recorder.is_terminated())

    def test_is_terminated_false_test2(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._writer.is_terminated.return_value = False
        video_recorder._uploader.is_terminated.return_value = True
        self.assertFalse(video_recorder.is_terminated())

    def test_is_terminated_false_test3(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._writer.is_terminated.return_value = False
        video_recorder._uploader.is_terminated.return_value = False
        self.assertFalse(video_recorder.is_terminated())

    def test_write(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder.write(1.0)
        video_recorder._subscriber.get.assert_called_once_with(timeout=1.0)
        video_recorder._writer.write.assert_called_once_with(frame=video_recorder._subscriber.get())

    def test_reset(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder.reset()
        video_recorder._writer.reset.assert_called_once()

    def test_upload(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder.upload()
        video_recorder._uploader.upload.assert_called_once()

    def test_successful_pause_at_run_state(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._state = VideoRecorderState.RUN
        video_recorder.pause()
        self.assertTrue(video_recorder._state == VideoRecorderState.PAUSE)
        video_recorder._subscriber.pause.assert_called_once()

    def test_failed_pause_at_wrong_state(self) -> None:
        with self.assertRaises(Exception):
            video_recorder = VideoRecorder(self.video_name,
                                           self.writer,
                                           self.uploader,
                                           self.subscriber)
            video_recorder._state = VideoRecorderState.PAUSE
            video_recorder.pause()

    def test_successful_unpause_at_pause_state(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._state = VideoRecorderState.PAUSE
        video_recorder.unpause()
        self.assertTrue(video_recorder._state == VideoRecorderState.RUN)
        video_recorder._subscriber.unpause.assert_called_once()

    def test_failed_unpause_at_wrong_state(self) -> None:
        with self.assertRaises(Exception):
            video_recorder = VideoRecorder(self.video_name,
                                           self.writer,
                                           self.uploader,
                                           self.subscriber)
            video_recorder._state = VideoRecorderState.RUN
            video_recorder.unpause()

    def test_failed_change_at_stop_state(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._state = VideoRecorderState.STOP
        with self.assertRaises(Exception):
            video_recorder.change(self.video_topic)

    def test_failed_change_at_pause_state(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._state = VideoRecorderState.PAUSE
        with self.assertRaises(Exception):
            video_recorder.change(self.video_topic)

    def test_successful_change(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._state = VideoRecorderState.RUN
        video_recorder.change(self.video_topic)
        video_recorder._subscriber.change.assert_called_once_with(self.video_topic)

    def test_successful_stop_at_run_state_non_immediate_stop(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._state = VideoRecorderState.RUN
        video_recorder.stop(should_immediate_stop=False)
        self.assertTrue(video_recorder._state == VideoRecorderState.STOP)
        video_recorder._subscriber.stop.assert_called_once()

    def test_successful_stop_at_run_state_immediate_stop(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._state = VideoRecorderState.RUN
        video_recorder.stop(should_immediate_stop=True)
        self.assertTrue(video_recorder._state == VideoRecorderState.STOP)
        video_recorder._subscriber.stop.assert_called_once()

    def test_successful_stop_at_pause_state_non_immediate_stop(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._state = VideoRecorderState.PAUSE
        video_recorder.stop(should_immediate_stop=False)
        self.assertTrue(video_recorder._state == VideoRecorderState.STOP)
        video_recorder._subscriber.stop.assert_called_once()

    def test_successful_stop_at_pause_state_immediate_stop(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        video_recorder._state = VideoRecorderState.PAUSE
        video_recorder.stop(should_immediate_stop=True)
        self.assertTrue(video_recorder._state == VideoRecorderState.STOP)
        video_recorder._subscriber.stop.assert_called_once()

    def test_failed_stop_at_stop_state(self) -> None:
        video_recorder = VideoRecorder(self.video_name,
                                       self.writer,
                                       self.uploader,
                                       self.subscriber)
        with self.assertRaises(Exception):
            video_recorder._state = VideoRecorderState.STOP
            video_recorder.stop(ignore_s3_upload=False, should_immediate_stop=False)
