import cv2

from unittest import TestCase
from unittest.mock import patch, MagicMock
from video_recorder.constants import VideoRecorderState
from video_recorder.video_writer import VideoWriter


class VideoWriterTest(TestCase):
    def setUp(self) -> None:
        self.video_name = "test"
        self.local_dir = "/home/ubuntu/Desktop/"
        self.video_format = ".mp4"
        self.max_num_frames = 100
        self.fourcc = cv2.VideoWriter_fourcc(*'avc1')
        self.fps = 15
        self.frame_width = 640
        self.frame_height = 480

    @patch.object(VideoWriter, "reset")
    def test_video_filename(self, reset_mock) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        self.assertTrue(video_writer.video_filename == "test_0.mp4")

    def test_video_index(self) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        video_writer._video_index = 1
        self.assertTrue(video_writer.video_index == 1)

    def test_should_immediate_stop(self) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        video_writer.should_immediate_stop = True
        self.assertTrue(video_writer.should_immediate_stop)

    def test_exceed_max_frames_true_bigger(self) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        video_writer._frame_index = 5
        video_writer._max_num_frames = 2
        self.assertTrue(video_writer.exceed_max_frames())

    def test_exceed_max_frames_true_equal(self) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        video_writer._frame_index = 2
        video_writer._max_num_frames = 2
        self.assertTrue(video_writer.exceed_max_frames())

    def test_exceed_max_frames_false(self) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        video_writer._frame_index = 1
        video_writer._max_num_frames = 2
        self.assertFalse(video_writer.exceed_max_frames())

    def test_is_terminated_true_1(self) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        video_writer.should_immediate_stop = False
        self.assertTrue(video_writer.is_terminated(VideoRecorderState.STOP, is_empty=True))

    def test_is_terminated_true_2(self) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        video_writer.should_immediate_stop = True
        self.assertTrue(video_writer.is_terminated(VideoRecorderState.STOP, is_empty=False))

    def test_is_terminated_false_1(self) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        video_writer.should_immediate_stop = True
        self.assertFalse(video_writer.is_terminated(VideoRecorderState.RUN, is_empty=True))

    def test_is_terminated_false_2(self) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        video_writer.should_immediate_stop = True
        self.assertFalse(video_writer.is_terminated(VideoRecorderState.PAUSE, is_empty=True))

    def test_is_terminated_false_3(self) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        video_writer.should_immediate_stop = False
        self.assertFalse(video_writer.is_terminated(VideoRecorderState.RUN, is_empty=False))

    def test_is_terminated_false_4(self) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        video_writer.should_immediate_stop = False
        self.assertFalse(video_writer.is_terminated(VideoRecorderState.PAUSE, is_empty=False))

    def test_is_terminated_false_5(self) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        video_writer.should_immediate_stop = False
        self.assertFalse(video_writer.is_terminated(VideoRecorderState.STOP, is_empty=False))

    @patch("video_recorder.video_writer.cv2")
    def test_write_without_resize(self, cv2_mock) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        frame = MagicMock()
        frame.shape = (640, 480, 3)
        video_writer._cv2_video_writer = MagicMock()
        video_writer.write(frame)
        cv2_mock.resize.assert_not_called()
        video_writer._cv2_video_writer.write.assert_called_once_with(frame)
        self.assertTrue(video_writer._frame_index == 1)

    @patch("video_recorder.video_writer.cv2")
    def test_write_with_resize(self, cv2_mock) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        frame = MagicMock()
        frame.shape = (100, 100, 3)
        video_writer._cv2_video_writer = MagicMock()
        video_writer.write(frame)
        cv2_mock.resize.assert_called_once_with(frame, (640, 480))
        video_writer._cv2_video_writer.write.assert_called_once_with(cv2_mock.resize.return_value)
        self.assertTrue(video_writer._frame_index == 1)

    @patch("video_recorder.video_writer.cv2")
    def test_reset_without_release(self, cv2_mock) -> None:
        video_writer = VideoWriter(
            video_name=self.video_name,
            local_dir=self.local_dir,
            video_format=self.video_format,
            max_num_frames=self.max_num_frames,
            fourcc=self.fourcc,
            fps=self.fps,
            frame_width=self.frame_width,
            frame_height=self.frame_height)
        video_writer._cv2_video_writer = MagicMock()
        self.assertTrue(video_writer._frame_index == 0)
        video_writer._cv2_video_writer.release.assert_not_called()
        cv2_mock.VideoWriter.assert_called_once_with(
            "/home/ubuntu/Desktop/test_0.mp4",
            self.fourcc,
            self.fps, (self.frame_width, self.frame_height))
        self.assertTrue(video_writer._video_index == 1)

    @patch("video_recorder.video_writer.cv2")
    def test_reset_with_release(self, cv2_mock) -> None:
        with self.assertRaises(Exception):
            video_writer = VideoWriter(
                video_name=self.video_name,
                local_dir=self.local_dir,
                video_format=self.video_format,
                max_num_frames=self.max_num_frames,
                fourcc=self.fourcc,
                fps=self.fps,
                frame_width=self.frame_width,
                frame_height=self.frame_height)
            video_writer._cv2_video_writer = MagicMock()
            video_writer._cv2_video_writer.release.side_effect = Exception()
            video_writer.reset()
