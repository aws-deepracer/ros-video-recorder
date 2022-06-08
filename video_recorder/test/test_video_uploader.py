from unittest import TestCase
from unittest.mock import patch, MagicMock
from video_recorder.video_uploader import VideoUploader
from video_recorder.constants import (S3_MAX_RETRY_ATTEMPTS,
                                      S3_RETRY_MODE,
                                      S3_RETRY_CONNECT_TIMEOUT)


class VideoUploaderTest(TestCase):
    def setUp(self) -> None:
        self.video_name = "test"
        self.local_dir = "/home/ubuntu/Desktop/"
        self.video_format = ".mp4"
        self.s3_bucket = "s3_bucket"
        self.s3_prefix = "s3_prefix"
        self.s3_region = "us-east-1"

    @patch("video_recorder.video_uploader.boto3")
    @patch("video_recorder.video_uploader.botocore")
    def test_init(self, botocore_mock, boto3_mock) -> None:
        VideoUploader(self.video_name,
                      self.local_dir,
                      self.video_format,
                      self.s3_bucket,
                      self.s3_prefix,
                      self.s3_region)

        boto3_mock.Session().client.assert_called_once_with(
            "s3",
            region_name=self.s3_region,
            config=botocore_mock.config.Config(retries=dict(max_attempts=S3_MAX_RETRY_ATTEMPTS,
                                                            mode=S3_RETRY_MODE),
                                               connect_timeout=S3_RETRY_CONNECT_TIMEOUT))

    def test_video_name(self) -> None:
        video_uploader = VideoUploader(self.video_name,
                                       self.local_dir,
                                       self.video_format,
                                       self.s3_bucket,
                                       self.s3_prefix,
                                       self.s3_region)
        self.assertTrue(video_uploader.video_filename == "test_0.mp4")

    def test_is_terminated_false_small_default(self) -> None:
        video_uploader = VideoUploader(self.video_name,
                                       self.local_dir,
                                       self.video_format,
                                       self.s3_bucket,
                                       self.s3_prefix,
                                       self.s3_region)
        video_uploader._video_index = 0
        self.assertFalse(video_uploader.is_terminated(target_video_index=2))

    def test_is_terminated_false_small_non_default_value(self) -> None:
        video_uploader = VideoUploader(self.video_name,
                                       self.local_dir,
                                       self.video_format,
                                       self.s3_bucket,
                                       self.s3_prefix,
                                       self.s3_region)
        video_uploader._video_index = 1
        self.assertFalse(video_uploader.is_terminated(target_video_index=2))

    def test_is_terminated_true_equal(self) -> None:
        video_uploader = VideoUploader(self.video_name,
                                       self.local_dir,
                                       self.video_format,
                                       self.s3_bucket,
                                       self.s3_prefix,
                                       self.s3_region)
        video_uploader._video_index = 2
        self.assertTrue(video_uploader.is_terminated(target_video_index=2))

    def test_is_terminated_true_large(self) -> None:
        video_uploader = VideoUploader(self.video_name,
                                       self.local_dir,
                                       self.video_format,
                                       self.s3_bucket,
                                       self.s3_prefix,
                                       self.s3_region)
        video_uploader._video_index = 3
        self.assertTrue(video_uploader.is_terminated(target_video_index=2))

    @patch("video_recorder.video_uploader.rospy")
    @patch("video_recorder.video_uploader.boto3")
    @patch("video_recorder.video_uploader.botocore")
    @patch("video_recorder.video_uploader.os")
    def test_upload_to_s3(self, os_mock, botocore_mock, boto3_mock, rospy_mock) -> None:
        video_uploader = VideoUploader(self.video_name,
                                       self.local_dir,
                                       self.video_format,
                                       self.s3_bucket,
                                       self.s3_prefix,
                                       self.s3_region)
        video_uploader._s3_client = MagicMock()
        video_uploader.upload()
        video_uploader._s3_client.upload_file.assert_called_once_with(
            Filename=os_mock.path.join(video_uploader._local_dir, video_uploader.video_filename),
            Bucket=self.s3_bucket,
            Key=os_mock.path.join(video_uploader._s3_prefix, video_uploader.video_filename))
        os_mock.remove.assert_called_once_with(
            os_mock.path.join(video_uploader._local_dir, video_uploader.video_filename))
        self.assertEqual(video_uploader._video_index, 1)

    @patch("video_recorder.video_uploader.rospy")
    @patch("video_recorder.video_uploader.boto3")
    @patch("video_recorder.video_uploader.botocore")
    @patch("video_recorder.video_uploader.os")
    def test_upload_not_to_s3(self, os_mock, botocore_mock, boto3_mock, rospy_mock) -> None:
        video_uploader = VideoUploader(self.video_name,
                                       self.local_dir,
                                       self.video_format,
                                       "",
                                       "",
                                       "")
        video_uploader.upload()
        os_mock.remove.assert_not_called()
        self.assertEqual(video_uploader._video_index, 1)
