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
"""Video Recorder Constants"""

import cv2

from enum import Enum, unique

# https://boto3.amazonaws.com/v1/documentation/api/latest/guide/retries.html for more retries info
# retry attempts with default 5 for legacy and 3 for standard
S3_MAX_RETRY_ATTEMPTS = 5
# retry mode with default as legacy
S3_RETRY_MODE = "standard"
# retry connect timeout with default as 60
S3_RETRY_CONNECT_TIMEOUT = 60

DEFAULT_VIDEO_FRAME_WIDTH = 640
DEFAULT_VIDEO_FRAME_HEIGHT = 480
DEFAULT_VIDEO_FOURCC = cv2.VideoWriter_fourcc(*'avc1')
DEFAULT_VIDEO_FPS = 15
DEFAULT_FORMAT = ".mp4"


class VideoRecorderServiceTopic():
    """
    Video recorder service topic
    """
    START = "video_recorder/start"
    PAUSE = "video_recorder/pause"
    UNPAUSE = "video_recorder/unpause"
    STOP = "video_recorder/stop"
    CHANGE = "video_recorder/change"


@unique
class VideoRecorderState(Enum):
    """
    Video recorder state
    """
    RUN = "RUN"
    PAUSE = "PAUSE"
    STOP = "STOP"
