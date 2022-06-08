from unittest import TestCase
from unittest.mock import patch
from video_recorder.video_recorder_node import main


@patch("video_recorder.video_recorder_node.VideoRecorderService")
@patch("video_recorder.video_recorder_node.rospy")
class VideoRecorderNodeTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_main(self, rospy_mock, video_service_mock) -> None:
        main()
        rospy_mock.init_node.assert_called_once_with(name="video_recorder",
                                                     anonymous=False,
                                                     log_level=rospy_mock.INFO)
        video_service_mock.assert_called_once()
        video_service_mock.return_value.start.assert_called_once()
        rospy_mock.spin.assert_called_once()
        video_service_mock.return_value.stop.assert_called_once()
