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
"""A class for Video Subscriber."""
import abc
import queue
import rospy
import numpy as np

from cv_bridge import CvBridge
from threading import Lock
from typing import Optional
from sensor_msgs.msg import Image
from video_recorder.video_exception import RosVideoError

# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class VideoSubscriberObserverInterface(ABC):
    """
    VideoSubscriberObserverInterface class
    """
    @abc.abstractmethod
    def on_frame_received(self, video_subscriber: "VideoSubscriber", frame: np.ndarray) -> None:
        """
        call back when frame is received

        Args:
            video_subscriber (VideoSubscriber): video_subscriber
            frame (np.ndarray): image frame ndarray
        """
        raise NotImplementedError('Observer must implement on_frame_received')


class VideoSubscriber(object):
    """
    VideoSubscriber class
    """

    def __init__(self,
                 video_name: str,
                 video_topic: str,
                 observer: VideoSubscriberObserverInterface) -> None:
        """
        VideoSubscriber class constructor

        Args:
            video_name (str): video namespace
            video_topic (str): video subscriber topic
            observer (VideoSubscriberObserverInterface): VideoSubscriberObserverInterface observer instance
        """
        # get_published_topics returns
        # list of topic names and types: [[topic1, type1]...[topicN, typeN]]
        if [video_topic, "sensor_msgs/Image"] not in rospy.get_published_topics():
            raise RosVideoError("[VideoSubscriber] video subscriber {} cannot "
                                "start to record because topic [{}, sensor_msgs/Image] "
                                "do not exist".format(
                                    video_name,
                                    video_topic))
        self._is_paused = False
        self._video_name = video_name
        self._video_topic = video_topic
        self._observer = observer
        self._cv2_bridge = CvBridge()
        self._subscriber_lock = Lock()
        self._frame_queue = queue.Queue()
        self._video_subscriber = rospy.Subscriber(
            video_topic,
            Image,
            self._subscriber_callback)
        self._curr_frame = None

    @property
    def video_name(self):
        """
        video name

        Returns:
            str: video name
        """
        return self._video_name

    def _subscriber_callback(self, msg_frame: Image) -> None:
        """
        video subscriber callback

        Args:
            msg_frame (Image): ROS image topic
        """
        lock_acquired = self._subscriber_lock.acquire(False)
        if lock_acquired:
            self._curr_frame = self._cv2_bridge.imgmsg_to_cv2(msg_frame, "bgr8")
            self._notify()
            self._subscriber_lock.release()

    def is_empty(self) -> bool:
        """
        is frame queue empty or not

        Returns:
            bool: True if frame queue is empty and False otherwise
        """
        return self._frame_queue.empty()

    def _notify(self) -> None:
        """
        notify observer a new frame is put into frame queue
        """
        self._frame_queue.put(self._curr_frame)
        self._observer.on_frame_received(self, self._curr_frame)

    def get(self, timeout: Optional[float] = None) -> np.ndarray:
        """
        Get one frame from frame queue

        Args:
            timeout (Optional[float]): timeout in second for get

        Returns:
            np.ndarray: image frame in np.ndarray
        """
        return self._frame_queue.get(timeout=timeout)

    def _pause(self) -> None:
        """
        Pause video subscriber by acquiring subscriber lock to bypass subscriber callback
        """
        if not self._is_paused:
            self._subscriber_lock.acquire()
            self._is_paused = True
            rospy.loginfo("[VideoSubscriber]: {} pause/stop on {} topic successfully".format(
                self._video_name, self._video_topic))
        else:
            raise RosVideoError("[VideoSubscriber] video subscriber {} cannot "
                                "pause/stop because it has already been"
                                "paused/stopped".format(self._video_name))

    def stop(self) -> None:
        """
        Stop video subscriber by acquiring subscriber lock to bypass subscriber callback
        and notify one more last time
        """
        self._pause()
        self._notify()

    def pause(self) -> None:
        """
        Pause video subscriber by acquiring subscriber lock to bypass subscriber callback
        """
        self._pause()

    def unpause(self) -> None:
        """
        Unpause video subscriber by release subscriber lock to enter subscriber callback
        """
        if self._is_paused:
            self._subscriber_lock.release()
            self._is_paused = False
            rospy.loginfo("[VideoSubscriber]: {} unpause on {} topic successfully".format(
                self._video_name, self._video_topic))
        else:
            raise RosVideoError("[VideoSubscriber] video subscriber {} cannot "
                                "unpause because it has already been"
                                "unpaused".format(self._video_name))

    def change(self, video_topic: str) -> None:
        """
        Change video subscriber by acquire lock, unregister callback, and release lock.

        Args:
            video_topic (str): video subscriber topic
        """
        if video_topic == self._video_topic:
            raise RosVideoError("[VideoSubscriber] video subscriber {} cannot "
                                "change to same topic {}".format(
                                    self._video_name,
                                    video_topic))
        if [video_topic, "sensor_msgs/Image"] not in rospy.get_published_topics():
            raise RosVideoError("[VideoSubscriber] video subscriber {} cannot "
                                "start to record because topic "
                                "[{}, sensor_msgs/Image] do not exist".format(
                                    self._video_name,
                                    video_topic))
        if self._is_paused:
            raise RosVideoError("[VideoSubscriber] video subscriber {} cannot "
                                "change at pause/stop".format(self._video_name))
        self._subscriber_lock.acquire()
        self._video_subscriber.unregister()
        self._video_subscriber = rospy.Subscriber(
            video_topic,
            Image,
            self._subscriber_callback)
        self._subscriber_lock.release()
        rospy.loginfo("[VideoSubscriber]: {} change from {} topic to {} topic successfully".format(
            self._video_name, self._video_topic, video_topic))
        self._video_topic = video_topic
