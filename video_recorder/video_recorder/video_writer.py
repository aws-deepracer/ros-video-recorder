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
"""A class for Video Writer."""
import cv2
import os

import numpy as np
from video_recorder.constants import (DEFAULT_VIDEO_FRAME_WIDTH,
                                      DEFAULT_VIDEO_FRAME_HEIGHT,
                                      DEFAULT_VIDEO_FOURCC,
                                      DEFAULT_VIDEO_FPS,
                                      DEFAULT_FORMAT,
                                      VideoRecorderState)


class VideoWriter():
    """
    VideoWriter class
    """

    def __init__(self,
                 video_name: str,
                 local_dir: str,
                 video_format: str,
                 max_num_frames: int,
                 fourcc: int,
                 fps: float,
                 frame_width: int,
                 frame_height: int) -> None:
        """
        VideoWriter constructor

        Check args used for VideoWriter at
        https://docs.opencv.org/3.4/dd/d9e/classcv_1_1VideoWriter.html

        Args:
            video_name (str): video namespace
            local_dir (str): video file local directory
            video_format (str): video file format
            max_num_frames (int): video writer maximum number of frame to save into mp4
                                 file. This value is default to inf. Developer should
                                 choose frames based on the available disk size.
            fourcc (int): 4-character code of codec used to compress the frames.
            fps (float): Frame rate of the created video stream
            frame_width (int): width of video frame
            frame_height (int): height of video frame

        """

        # frame param
        self._frame_index = 0
        self._max_num_frames = max_num_frames if max_num_frames > 0 else float(
            'inf')
        self._should_immediate_stop = False

        # video param
        self._video_index = 0
        self._video_name = video_name
        self._video_format = video_format if video_format else DEFAULT_FORMAT
        self._local_dir = local_dir

        # cv2.VideoWriter params
        self._fourcc = fourcc if fourcc > 0 else DEFAULT_VIDEO_FOURCC
        self._fps = fps if fps > 0.0 else DEFAULT_VIDEO_FPS
        self._frame_width = frame_width if frame_width > 0 else DEFAULT_VIDEO_FRAME_WIDTH
        self._frame_height = frame_height if frame_height > 0 else DEFAULT_VIDEO_FRAME_HEIGHT
        self._frame_size = (self._frame_width, self._frame_height)
        self._cv2_video_writer = None

        self.reset()

    @property
    def video_filename(self) -> str:
        """
        Video filename getter

        Returns:
            str: string of video filename
        """
        return self._video_name + "_" + str(self._video_index) + self._video_format

    @property
    def video_index(self) -> int:
        """
        video index getter

        Returns:
            int: video index
        """
        return self._video_index

    @property
    def should_immediate_stop(self) -> bool:
        """
        should immediate stop getter

        Returns:
            bool: True if should immediate stop and False otherwise
        """
        return self._should_immediate_stop

    @should_immediate_stop.setter
    def should_immediate_stop(self, val: bool) -> None:
        """
        should immediate stop putter

        Args:
            val (bool): should immediate stop value in bool
        """
        self._should_immediate_stop = val

    def exceed_max_frames(self) -> bool:
        """
        exceed the maximum number of frames

        Returns:
            bool: True if maximum number of frames has exceed and False otherwise
        """
        return self._frame_index >= self._max_num_frames

    def is_terminated(self, recorder_state: VideoRecorderState, is_empty: bool) -> bool:
        """
        is VideoWriter terminated

        Args:
            recorder_state (VideoRecorderState): VideoRecorderState state
            is_empty (bool): True if frame queue is empty and False otherwise

        Returns:
            bool: True if VideoWriter has finished life cycle and False otherwise.
        """
        if recorder_state == VideoRecorderState.STOP:
            if is_empty or self.should_immediate_stop:
                return True
        return False

    def write(self, frame: np.ndarray) -> None:
        """
        Write video frame into local file by getting frame from queue
        and increment frame index count

        Args:
            frame(np.ndarray): cv2 video frame represented as numpy arrays

        Returns:
            bool: True is queue is empty before write and False otherwise
        """
        width, height, _ = frame.shape
        if (width, height) != self._frame_size:
            frame = cv2.resize(frame, self._frame_size)
        self._cv2_video_writer.write(frame)
        self._frame_index += 1

    def reset(self) -> None:
        """
        Reset video writer
        """
        if self._cv2_video_writer:
            self._cv2_video_writer.release()
        self._frame_index = 0
        self._cv2_video_writer = cv2.VideoWriter(
            os.path.join(self._local_dir, self.video_filename),
            self._fourcc,
            self._fps,
            self._frame_size)
        self._video_index += 1
