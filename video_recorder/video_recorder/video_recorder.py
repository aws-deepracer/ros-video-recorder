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
"""A class for Video Recorder."""
from threading import RLock
from typing import Optional
from video_recorder.constants import VideoRecorderState
from video_recorder.video_writer import VideoWriter
from video_recorder.video_uploader import VideoUploader
from video_recorder.video_subscriber import VideoSubscriber
from video_recorder.video_exception import RosVideoError


class VideoRecorder(object):
    """
    VideoRecorder class
    """

    def __init__(self,
                 video_name: str,
                 writer: VideoWriter,
                 uploader: VideoUploader,
                 subscriber: VideoSubscriber) -> None:
        """
        VideoRecorder constructor

        Args:
            video_name (str): video recorder namespace
            writer (VideoWriter): VideoWriter class instance
            uploader (VideoUploader): VideoUploader class instance
            subscriber (VideoSubscriber): VideoSubscriber class instance
        """
        self._lock = RLock()
        self._video_name = video_name
        self._writer = writer
        self._uploader = uploader
        self._subscriber = subscriber
        self._state = VideoRecorderState.RUN

    @property
    def state(self) -> VideoRecorderState:
        """
        state getter

        Returns:
            VideoRecorderState: video recorder state
        """
        with self._lock:
            return self._state

    @property
    def video_name(self) -> str:
        """
        video name getter

        Returns:
            str: vidoe name
        """
        return self._video_name

    def is_empty(self) -> bool:
        """
        is VideoSubscriber class instance frame queue empty or not

        Returns:
            bool: True if frame queue is empty and False otherwise
        """
        return self._subscriber.is_empty()

    def exceed_max_frames(self) -> bool:
        """
        exceed the maximum number of frames

        Returns:
            bool: True if maximum number of frames has exceed and False otherwise
        """
        return self._writer.exceed_max_frames()

    def should_immediate_stop(self) -> bool:
        """
        should immediate stop

        Returns:
            bool: True if should immediate stop and False otherwise
        """
        return self._writer.should_immediate_stop

    def is_terminated(self) -> bool:
        """
        check whether VideoRecorder class instance has terminated its life cycle
        based on writer and uploader

        Returns:
            bool: True if life cycle has terminated and False otherwise
        """
        with self._lock:
            return self._writer.is_terminated(recorder_state=self._state,
                                              is_empty=self.is_empty()) \
                and self._uploader.is_terminated(target_video_index=self._writer.video_index)

    def write(self, timeout: Optional[float] = None) -> None:
        """
        video recorder VideoWriter class instance to write a frame

        Args:
            timeout (Optional[float]): timeout in second for frame get
        """
        frame = self._subscriber.get(timeout=timeout)
        self._writer.write(frame=frame)

    def reset(self) -> None:
        """
        video recorder VideoWriter class instance to reset by releasing
        the current cv2.VideoWriter and start a new one
        """
        self._writer.reset()

    def upload(self) -> None:
        """
        video recorder VideoUploader class instance to upload local file
        to s3 bucket
        """
        self._uploader.upload()

    def pause(self) -> None:
        """
        Pause video recorder
        """
        with self._lock:
            if self._state == VideoRecorderState.RUN:
                self._state = VideoRecorderState.PAUSE
                self._subscriber.pause()
            else:
                raise RosVideoError("[VideoRecorder] video recorder {} is "
                                    "at {} which cannot be paused.".format(
                                        self._video_name,
                                        self._state))

    def unpause(self) -> None:
        """
        Unpause video recorder
        """
        with self._lock:
            if self._state == VideoRecorderState.PAUSE:
                self._state = VideoRecorderState.RUN
                self._subscriber.unpause()
            else:
                raise RosVideoError("[VideoRecorder] video recorder {} is "
                                    "at {} which cannot be unpaused.".format(
                                        self._video_name,
                                        self._state))

    def change(self, video_topic: str) -> None:
        """
        Change video recorder to a new topic

        Args:
            video_topic (str): ROS topic
        """
        with self._lock:
            if self._state in [VideoRecorderState.STOP, VideoRecorderState.PAUSE]:
                raise RosVideoError("[VideoRecorder] video recorder {} is "
                                    "at {} which cannot be changed.".format(
                                        self._video_name,
                                        self._state))
            self._subscriber.change(video_topic)

    def stop(self, should_immediate_stop: bool) -> None:
        """
        Stop video recorder

        video_subscriber will notify observer video_recorder_service one more time
        during stop to guarantee enqueue uploader_queue during stop. Otherwise,
        during stop if writer_queue has already been empty, enqueue uploader_queue will never
        happen.

        Args:
            should_immediate_stop (bool): true if immediately upload to s3 bucket without flushing
                                          remaining frames in queue prior to stop
                                          and false otherwise.
        """
        with self._lock:
            if self._state == VideoRecorderState.RUN:
                self._writer.should_immediate_stop = should_immediate_stop
                self._state = VideoRecorderState.STOP
                self._subscriber.stop()
            elif self._state == VideoRecorderState.PAUSE:
                self._writer.should_immediate_stop = should_immediate_stop
                self._state = VideoRecorderState.STOP
                self._subscriber.stop()
            else:
                raise RosVideoError("[VideoRecorder] video recorder {} is "
                                    "at {} which cannot be stopped.".format(
                                        self._video_name,
                                        self._state))
