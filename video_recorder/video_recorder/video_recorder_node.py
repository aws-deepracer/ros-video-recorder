#!/usr/bin/env python3

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
"""A class for Video Recorder Node."""
import rospy

from video_recorder.video_recorder_service import VideoRecorderService


def main():
    rospy.init_node(name="video_recorder",
                    anonymous=False,
                    log_level=rospy.INFO)
    video_recorder_service = VideoRecorderService()
    video_recorder_service.start()
    rospy.spin()
    video_recorder_service.stop()


if __name__ == '__main__':
    main()
