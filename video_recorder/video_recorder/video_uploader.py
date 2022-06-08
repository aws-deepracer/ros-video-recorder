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
"""A class for Video Uploader."""
import botocore
import boto3
import os
import rospy

from video_recorder.constants import (S3_MAX_RETRY_ATTEMPTS,
                                      S3_RETRY_MODE,
                                      S3_RETRY_CONNECT_TIMEOUT,
                                      DEFAULT_FORMAT)


class VideoUploader(object):
    """
    VideoUploader class
    """

    def __init__(self,
                 video_name: str,
                 local_dir: str,
                 video_format: str,
                 s3_bucket: str,
                 s3_prefix: str,
                 s3_region: str) -> None:
        """
        VideoUploader constructor

        Args:
            video_name (str): video namespace
            local_dir (str): video file local directory
            video_format (str): video file format
            s3_bucket (str): s3 bucket to upload mp4 file
            s3_prefix (str): s3 prefix to upload mp4 file
            s3_region (str): s3 region to upload mp4 file
        """
        self._video_index = 0
        self._video_name = video_name
        self._local_dir = local_dir
        self._video_format = video_format if video_format else DEFAULT_FORMAT
        self._s3_bucket = s3_bucket
        self._s3_prefix = s3_prefix
        self._s3_region = s3_region

        self._s3_client = None
        if self._s3_bucket and self._s3_prefix and self._s3_region:
            self._s3_client = boto3.Session().client(
                "s3",
                region_name=self._s3_region,
                config=botocore.config.Config(retries=dict(max_attempts=S3_MAX_RETRY_ATTEMPTS,
                                                           mode=S3_RETRY_MODE),
                                              connect_timeout=S3_RETRY_CONNECT_TIMEOUT))

    @property
    def video_filename(self) -> str:
        """
        Video filename getter

        Returns:
            str: string of video filename
        """
        return self._video_name + "_" + str(self._video_index) + self._video_format

    def is_terminated(self, target_video_index: int) -> bool:
        """
        is VideoUploader should be terminated by uploading all required videos

        Args:
            target_video_index (int): get video index

        Returns:
            bool: True if VideoUploader has finished life cycle to upload all videos
                  and False otherwise.
        """
        return self._video_index >= target_video_index

    def upload(self) -> None:
        """
        Upload local video to s3 bucket if s3 client exist.
        """
        if self._s3_client:
            local_file_path = os.path.join(self._local_dir, self.video_filename)
            s3_file_path = os.path.join(self._s3_prefix, self.video_filename)
            try:
                self._s3_client.upload_file(
                    Filename=local_file_path,
                    Bucket=self._s3_bucket,
                    Key=s3_file_path)
                os.remove(os.path.join(self._local_dir, self.video_filename))
                rospy.loginfo("[VideoUploader]: {} video uploader to upload {} to {} "
                              "at {} bucket successfully".format(self._video_name,
                                                                 local_file_path,
                                                                 s3_file_path,
                                                                 self._s3_bucket))
            except botocore.exceptions.ClientError as ex:
                rospy.loginfo("[VideoUploader]: {} video uploader failed to upload: "
                              "{}".format(self._video_name, ex))
            except Exception as ex:
                rospy.loginfo("[VideoUploader]: {} video uploader failed to upload: "
                              "{}".format(self._video_name, ex))
        self._video_index += 1
