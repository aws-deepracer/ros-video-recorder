from unittest import TestCase
from unittest.mock import patch, call, MagicMock
from sensor_msgs.msg import Image
from video_recorder.video_subscriber import VideoSubscriber


@patch("video_recorder.video_subscriber.rospy")
class VideoSubscriberTest(TestCase):
    def setUp(self) -> None:
        self.observer = MagicMock()
        self.video_name = "test"
        self.video_topic = "test"

    def test_successful_init(self, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        rospy_mock.Subscriber.assert_called_once_with(
            self.video_topic,
            Image,
            video_subscriber._subscriber_callback)
        self.assertFalse(video_subscriber._is_paused)

    def test_video_name(self, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        self.assertTrue(video_subscriber.video_name == self.video_name)

    def test_failed_init_not_existing_image_topic(self, rospy_mock) -> None:
        with self.assertRaises(Exception):
            rospy_mock.get_published_topics.return_value = []
            VideoSubscriber(
                self.video_name,
                self.video_topic,
                self.observer)

    @patch.object(VideoSubscriber, "_notify")
    def test_subscriber_callback_lock_acquired(self, notify_mock, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber._subscriber_lock = MagicMock()
        video_subscriber._cv2_bridge = MagicMock()
        msg_frame = MagicMock()

        imgmsg_to_cv2_return_value = "imgmsg_to_cv2_return_value"
        video_subscriber._cv2_bridge.imgmsg_to_cv2.return_value = imgmsg_to_cv2_return_value
        video_subscriber._subscriber_lock.acquire.return_value = True

        video_subscriber._subscriber_callback(msg_frame)
        video_subscriber._subscriber_lock.acquire.assert_called_once()
        video_subscriber._cv2_bridge.imgmsg_to_cv2.assert_called_once_with(
            msg_frame, "bgr8")
        notify_mock.assert_called_once()
        video_subscriber._subscriber_lock.release.assert_called_once()

    @patch.object(VideoSubscriber, "_notify")
    def test_subscriber_callback_lock_not_acquired(self, notify_mock, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber._subscriber_lock = MagicMock()
        video_subscriber._cv2_bridge = MagicMock()
        msg_frame = MagicMock()

        imgmsg_to_cv2_return_value = "imgmsg_to_cv2_return_value"
        video_subscriber._cv2_bridge.imgmsg_to_cv2.return_value = imgmsg_to_cv2_return_value
        video_subscriber._subscriber_lock.acquire.return_value = False

        video_subscriber._subscriber_callback(msg_frame)
        video_subscriber._subscriber_lock.acquire.assert_called_once()
        video_subscriber._cv2_bridge.imgmsg_to_cv2.assert_not_called()
        notify_mock.assert_not_called()
        video_subscriber._subscriber_lock.release.assert_not_called()

    def test_is_empty(self, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber._frame_queue = MagicMock()
        video_subscriber.is_empty()
        video_subscriber._frame_queue.empty.assert_called_once()

    def test_notify(self, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber._frame_queue = MagicMock()
        video_subscriber._frame_queue.empty.return_value = True
        video_subscriber._curr_frame = "test"
        video_subscriber._notify()
        video_subscriber._frame_queue.put.assert_called_once_with(video_subscriber._curr_frame)
        video_subscriber._observer.on_frame_received.assert_called_once_with(video_subscriber, "test")

    def test_get(self, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber._frame_queue = MagicMock()
        video_subscriber.get(1.0)
        video_subscriber._frame_queue.get.assert_called_once_with(timeout=1.0)

    def test_private_pause_not_paused(self, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber._subscriber_lock = MagicMock()
        video_subscriber._is_paused = False
        video_subscriber._pause()
        video_subscriber._subscriber_lock.acquire.assert_called_once()
        self.assertTrue(video_subscriber._is_paused)

    def test_private_pause_already_paused(self, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber._subscriber_lock = MagicMock()
        video_subscriber._is_paused = True
        with self.assertRaises(Exception):
            video_subscriber._pause()

    @patch.object(VideoSubscriber, "_notify")
    @patch.object(VideoSubscriber, "_pause")
    def test_stop(self, pause_mock, notify_mock, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber.stop()
        notify_mock.assert_called_once()
        pause_mock.assert_called_once()

    @patch.object(VideoSubscriber, "_pause")
    def test_pause(self, pause_mock, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber.pause()
        pause_mock.assert_called_once()

    def test_unpause(self, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber._is_paused = True
        video_subscriber._subscriber_lock = MagicMock()
        video_subscriber.unpause()

    def test_unpause_already_unpaused(self, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber._subscriber_lock = MagicMock()
        video_subscriber._is_paused = False
        with self.assertRaises(Exception):
            video_subscriber.unpause()

    def test_failed_change_to_same_topic(self, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber._video_topic = self.video_topic
        with self.assertRaises(Exception):
            video_subscriber.change(self.video_topic)

    def test_failed_change_non_image_topic(self, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [
            [self.video_topic, "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        rospy_mock.get_published_topics.return_value = [[self.video_topic, 'blabla']]
        with self.assertRaises(Exception):
            video_subscriber.change(self.video_topic)

    def test_failed_change_at_pause(self, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"],
                                                        [self.video_topic + "1", "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber._is_paused = True
        with self.assertRaises(Exception):
            video_subscriber.change(self.video_topic + "1")

    def test_successful_change(self, rospy_mock) -> None:
        rospy_mock.get_published_topics.return_value = [[self.video_topic, "sensor_msgs/Image"],
                                                        [self.video_topic + "1", "sensor_msgs/Image"]]
        video_subscriber = VideoSubscriber(
            self.video_name,
            self.video_topic,
            self.observer)
        video_subscriber._subscriber_lock = MagicMock()
        video_subscriber._video_subscriber = MagicMock()
        video_subscriber.change(self.video_topic + "1")
        video_subscriber._subscriber_lock.acquire.assert_called_once()
        video_subscriber._subscriber_lock.release.assert_called_once()
        # one from constructor and one from change command
        rospy_mock.Subscriber.has_calls(
            [call(self.video_topic,
                  Image,
                  video_subscriber._subscriber_callback),
             call(self.video_topic + "1",
                  Image,
                  video_subscriber._subscriber_callback)])
        self.assertTrue(video_subscriber._video_topic == self.video_topic + "1")
