import cv2

from unittest import TestCase
from unittest.mock import patch, MagicMock, call

from video_recorder.constants import (VideoRecorderServiceTopic,
                                      VideoRecorderState)
from video_recorder_msgs.srv import (
    VideoRecorderStartSrv,
    VideoRecorderStartSrvRequest,
    VideoRecorderPauseSrv,
    VideoRecorderPauseSrvRequest,
    VideoRecorderUnpauseSrv,
    VideoRecorderUnpauseSrvRequest,
    VideoRecorderStopSrv,
    VideoRecorderStopSrvRequest,
    VideoRecorderChangeSrv,
    VideoRecorderChangeSrvRequest)
from video_recorder.video_recorder_service import VideoRecorderService


@patch("video_recorder.video_recorder_service.rospy")
@patch("video_recorder.video_recorder_service.Thread")
@patch("video_recorder.video_recorder_service.VideoRecorder")
class VideoRecorderServiceTest(TestCase):
    def setUp(self) -> None:
        pass

    @patch("video_recorder.video_recorder_service.RLock")
    def test_init(self, rlock_mock, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        rlock_mock.assert_called_once()
        self.assertTrue(len(video_recorder_service._recorders_dict) == 0)
        self.assertTrue(video_recorder_service._uploader_queue.empty())
        self.assertTrue(video_recorder_service._writer_queue.empty())
        thread_mock.has_calls([call(target=video_recorder_service._upload),
                               call(target=video_recorder_service._write)])

    def test_on_frame_received_existing_name_put_successfully(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._lock = MagicMock()
        video_recorder_service._writer_queue = MagicMock()
        video_recorder_service._recorders_dict["test"] = "test_video_recorder"
        video_subscriber = MagicMock()
        video_subscriber.video_name = "test"
        curr_frame = "test"
        video_recorder_service.on_frame_received(video_subscriber=video_subscriber, frame=curr_frame)
        video_recorder_service._lock.__enter__.assert_called_once()
        video_recorder_service._writer_queue.put.assert_called_once_with("test_video_recorder")
        video_recorder_service._lock.__exit__.assert_called_once()

    def test_on_frame_received_non_existing_name(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._lock = MagicMock()
        video_recorder_service._writer_queue = MagicMock()
        video_subscriber = MagicMock()
        video_subscriber.video_name = "test"
        curr_frame = "test"
        video_recorder_service.on_frame_received(video_subscriber=video_subscriber, frame=curr_frame)
        video_recorder_service._lock.__enter__.assert_called_once()
        video_recorder_service._writer_queue.put.assert_not_called()
        video_recorder_service._lock.__exit__.assert_called_once()

    def test_start(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service.start()
        rospy_mock.has_calls([call(VideoRecorderServiceTopic.START,
                                   VideoRecorderStartSrv,
                                   video_recorder_service._start_callback),
                              call(VideoRecorderServiceTopic.PAUSE,
                                   VideoRecorderPauseSrv,
                                   video_recorder_service._pause_callback),
                              call(VideoRecorderServiceTopic.UNPAUSE,
                                   VideoRecorderUnpauseSrv,
                                   video_recorder_service._unpause_callback),
                              call(VideoRecorderServiceTopic.STOP,
                                   VideoRecorderStopSrv,
                                   video_recorder_service._stop_callback),
                              call(VideoRecorderServiceTopic.CHANGE,
                                   VideoRecorderChangeSrv,
                                   video_recorder_service._change_callback)])
        thread_mock.start.has_calls([call(),
                                     call()])

    def test_stop(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._writer_thread = MagicMock()
        video_recorder_service._uploader_thread = MagicMock()
        video_recorder_service._video_recorder_start_srv = MagicMock()
        video_recorder_service._video_recorder_pause_srv = MagicMock()
        video_recorder_service._video_recorder_unpause_srv = MagicMock()
        video_recorder_service._video_recorder_stop_srv = MagicMock()
        video_recorder_service._video_recorder_change_srv = MagicMock()
        video_recorder_service._recorders_dict["test"] = video_recorder_mock
        video_recorder_service._is_service_running = True

        video_recorder_service.stop()
        video_recorder_service._video_recorder_start_srv.shutdown.assert_called_once()
        video_recorder_service._video_recorder_pause_srv.shutdown.assert_called_once()
        video_recorder_service._video_recorder_unpause_srv.shutdown.assert_called_once()
        video_recorder_service._video_recorder_stop_srv.shutdown.assert_called_once()
        video_recorder_service._video_recorder_change_srv.shutdown.assert_called_once()
        video_recorder_mock.stop.assert_called_once_with(should_immediate_stop=True)
        video_recorder_service._writer_thread.join.assert_called_once()
        video_recorder_service._uploader_thread.join.assert_called_once()

    @patch("video_recorder.video_recorder_service.VideoUploader")
    @patch("video_recorder.video_recorder_service.VideoWriter")
    @patch("video_recorder.video_recorder_service.VideoSubscriber")
    @patch("video_recorder.video_recorder_service.os")
    def test_successful_start_callback(
            self,
            os_mock,
            video_subscriber_mock,
            video_writer_mock,
            video_uploader_mock,
            video_recorder_mock,
            thread_mock,
            rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._lock = MagicMock()
        os_mock.path.isdir.return_value = True
        request = VideoRecorderStartSrvRequest()
        request.video_name = "test"
        request.local_dir = "/home/ubuntu/Desktop/"
        request.video_topic = "test"
        request.video_format = ".mp4"
        request.max_num_frames = 100
        request.fourcc = cv2.VideoWriter_fourcc(*'avc1')
        request.fps = 15
        request.frame_width = 640
        request.frame_height = 480
        request.s3_bucket = "test"
        request.s3_prefix = "test"
        request.s3_region = "us-east-1"
        response = video_recorder_service._start_callback(request)
        video_recorder_service._lock.__enter__.assert_called_once()
        video_writer_mock.assert_called_once_with(
            video_name=request.video_name,
            local_dir=request.local_dir,
            video_format=request.video_format,
            max_num_frames=request.max_num_frames,
            fourcc=request.fourcc,
            fps=request.fps,
            frame_width=request.frame_width,
            frame_height=request.frame_height)
        video_uploader_mock.assert_called_once_with(
            video_name=request.video_name,
            local_dir=request.local_dir,
            video_format=request.video_format,
            s3_bucket=request.s3_bucket,
            s3_prefix=request.s3_prefix,
            s3_region=request.s3_region)
        video_subscriber_mock.assert_called_once_with(
            video_name=request.video_name,
            video_topic=request.video_topic,
            observer=video_recorder_service)
        video_recorder_mock.assert_called_with(
            video_name=request.video_name,
            writer=video_writer_mock.return_value,
            uploader=video_uploader_mock.return_value,
            subscriber=video_subscriber_mock.return_value)
        self.assertTrue(response.success)
        video_recorder_service._lock.__exit__.assert_called_once()

    def test_failed_start_callback_name_in_use(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service.notify = MagicMock()
        request = VideoRecorderStartSrvRequest()
        request.video_name = "test"
        request.local_dir = "/home/ubuntu/Desktop/"
        request.video_topic = "test"
        request.max_num_frames = 100
        request.fourcc = cv2.VideoWriter_fourcc(*'avc1')
        request.fps = 15
        request.frame_width = 640
        request.frame_height = 480
        request.s3_bucket = "test"
        request.s3_prefix = "test"
        request.s3_region = "us-east-1"
        # video_name already in use
        video_recorder_service._recorders_dict[request.video_name] = "blabla"
        response = video_recorder_service._start_callback(request)
        video_recorder_mock.assert_not_called()
        self.assertFalse(response.success)

    def test_failed_start_callback_exception(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        with self.assertRaises(Exception):
            video_recorder_service = VideoRecorderService()
            video_recorder_service._start_callback.side_effect = Exception()
            video_recorder_service.notify = MagicMock()
            request = VideoRecorderStartSrvRequest()
            request.video_name = "test"
            request.local_dir = "/home/ubuntu/Desktop/"
            request.video_topic = "test"
            request.max_num_frames = 100
            request.fourcc = cv2.VideoWriter_fourcc(*'avc1')
            request.fps = 15
            request.frame_width = 640
            request.frame_height = 480
            request.s3_bucket = "test"
            request.s3_prefix = "test"
            request.s3_region = "us-east-1"
            response = video_recorder_service._start_callback(request)
            video_recorder_mock.assert_not_called()
            self.assertFalse(response.success)

    def test_successful_pause_callback(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        request = VideoRecorderPauseSrvRequest()
        request.video_name = "test"
        video_recorder_service._recorders_dict[request.video_name] = video_recorder_mock
        response = video_recorder_service._pause_callback(request)
        video_recorder_mock.pause.assert_called_once()
        self.assertTrue(response.success)

    def test_failed_pause_callback_name_not_in_use(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        request = VideoRecorderPauseSrvRequest()
        request.video_name = "test"
        response = video_recorder_service._pause_callback(request)
        video_recorder_mock.pause.assert_not_called()
        self.assertFalse(response.success)

    def test_failed_pause_callback_exception(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        with self.assertRaises(Exception):
            video_recorder_service = VideoRecorderService()
            request = VideoRecorderPauseSrvRequest()
            request.video_name = "test"
            video_recorder_service._pause_callback.side_effect = Exception()
            response = video_recorder_service._pause_callback(request)
            self.assertFalse(response.success)

    def test_successful_unpause_callback(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        request = VideoRecorderUnpauseSrvRequest()
        request.video_name = "test"
        video_recorder_service._recorders_dict[request.video_name] = video_recorder_mock
        response = video_recorder_service._unpause_callback(request)
        video_recorder_mock.unpause.assert_called_once()
        self.assertTrue(response.success)

    def test_failed_unpause_callback_name_not_in_use(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        request = VideoRecorderUnpauseSrvRequest()
        request.video_name = "test"
        response = video_recorder_service._unpause_callback(request)
        video_recorder_mock.unpause.assert_not_called()
        self.assertFalse(response.success)

    def test_failed_unpause_callback_exception(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        with self.assertRaises(Exception):
            video_recorder_service = VideoRecorderService()
            request = VideoRecorderUnpauseSrvRequest()
            request.video_name = "test"
            video_recorder_service._unpause_callback.side_effect = Exception()
            response = video_recorder_service._unpause_callback()
            video_recorder_mock.unpause.assert_not_called()
            self.assertFalse(response.success)

    def test_successful_change_callback(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        request = VideoRecorderChangeSrvRequest()
        request.video_name = "test"
        video_recorder_service._recorders_dict[request.video_name] = video_recorder_mock
        response = video_recorder_service._change_callback(request)
        video_recorder_mock.change.assert_called_once()
        self.assertTrue(response.success)

    def test_failed_change_callback_name_not_in_use(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        request = VideoRecorderChangeSrvRequest()
        request.video_name = "test"
        response = video_recorder_service._change_callback(request)
        video_recorder_mock.change.assert_not_called()
        self.assertFalse(response.success)

    def test_failed_change_callback_exception(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        with self.assertRaises(Exception):
            video_recorder_service = VideoRecorderService()
            request = VideoRecorderChangeSrvRequest()
            request.video_name = "test"
            video_recorder_service._unpause_callback.side_effect = Exception()
            response = video_recorder_service._change_callback()
            self.assertFalse(response.success)

    def test_successful_stop_callback(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        request = VideoRecorderStopSrvRequest()
        request.video_name = "test"
        video_recorder_service._recorders_dict[request.video_name] = video_recorder_mock
        response = video_recorder_service._stop_callback(request)
        video_recorder_mock.stop.assert_called_once()
        self.assertTrue(response.success)

    def test_failed_stop_callback_name_not_in_use(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        request = VideoRecorderStopSrvRequest()
        request.video_name = "test"
        response = video_recorder_service._stop_callback(request)
        video_recorder_mock.stop.assert_not_called()
        self.assertFalse(response.success)

    def test_failed_stop_callback_exception(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        with self.assertRaises(Exception):
            video_recorder_service = VideoRecorderService()
            request = VideoRecorderStopSrvRequest()
            request.video_name = "test"
            video_recorder_service._unpause_callback.side_effect = Exception()
            response = video_recorder_service._stop_callback()
            self.assertFalse(response.success)

    def test_enqueue_writer_frame_queue_empty(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._writer_queue = MagicMock()
        video_recorder_mock.is_empty.return_value = True
        self.assertFalse(video_recorder_service._enqueue_writer(video_recorder_mock))
        video_recorder_service._writer_queue.put.assert_not_called()

    def test_enqueue_writer_frame_queue_not_empty(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._writer_queue = MagicMock()
        video_recorder_mock.is_empty.return_value = False
        self.assertTrue(video_recorder_service._enqueue_writer(video_recorder_mock))
        video_recorder_service._writer_queue.put.assert_called_once_with(video_recorder_mock)

    def test_enqueue_uploader(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._uploader_queue = MagicMock()
        video_recorder_service._enqueue_uploader(video_recorder_mock)
        video_recorder_mock.reset.assert_called_once()
        video_recorder_service._uploader_queue.put.assert_called_once_with(video_recorder_mock)

    def test_write_exceed_max_frames(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._enqueue_writer = MagicMock()
        video_recorder_service._writer_finished_event = MagicMock()
        video_recorder_service._enqueue_uploader = MagicMock()
        video_recorder_service._uploader_queue = MagicMock()
        video_recorder_service._writer_queue = MagicMock()
        video_recorder_service._is_service_running = False
        video_recorder_service._writer_queue.empty.side_effect = [False, True]
        video_recorder_service._writer_queue.get.return_value = video_recorder_mock
        video_recorder_mock.exceed_max_frames.return_value = True
        video_recorder_service._write()
        video_recorder_service._writer_queue.get.assert_called_once()
        video_recorder_mock.write.assert_called_once()
        video_recorder_service._enqueue_uploader.assert_called_once_with(video_recorder_mock)
        video_recorder_service._writer_finished_event.set.assert_called_once()

    def test_write_not_exceed_max_frames(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._enqueue_writer = MagicMock()
        video_recorder_service._enqueue_uploader = MagicMock()
        video_recorder_service._uploader_queue = MagicMock()
        video_recorder_service._writer_queue = MagicMock()
        video_recorder_service._is_service_running = False
        video_recorder_service._writer_queue.empty.side_effect = [False, True]
        video_recorder_service._writer_queue.get.return_value = video_recorder_mock
        video_recorder_mock.exceed_max_frames.return_value = False
        video_recorder_service._write()
        video_recorder_service._writer_queue.get.assert_called_once()
        video_recorder_mock.write.assert_called_once()
        video_recorder_service._enqueue_uploader.assert_not_called()

    def test_write_run_state(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._enqueue_writer = MagicMock()
        video_recorder_service._enqueue_uploader = MagicMock()
        video_recorder_service._uploader_queue = MagicMock()
        video_recorder_service._writer_queue = MagicMock()
        video_recorder_service._is_service_running = False
        video_recorder_service._writer_queue.empty.side_effect = [False, True]
        video_recorder_service._writer_queue.get.return_value = video_recorder_mock
        video_recorder_mock.exceed_max_frames.return_value = False
        video_recorder_mock.state = VideoRecorderState.RUN
        video_recorder_service._write()
        video_recorder_service._enqueue_writer.assert_called_once_with(video_recorder_mock)

    def test_write_pause_state(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._enqueue_writer = MagicMock()
        video_recorder_service._enqueue_uploader = MagicMock()
        video_recorder_service._uploader_queue = MagicMock()
        video_recorder_service._writer_queue = MagicMock()
        video_recorder_service._is_service_running = False
        video_recorder_service._writer_queue.empty.side_effect = [False, True]
        video_recorder_service._writer_queue.get.return_value = video_recorder_mock
        video_recorder_mock.exceed_max_frames.return_value = False
        video_recorder_mock.state = VideoRecorderState.PAUSE
        video_recorder_service._write()
        video_recorder_service._enqueue_writer.assert_called_once_with(video_recorder_mock)

    def test_write_stop_state_immediate_stop(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._enqueue_writer = MagicMock()
        video_recorder_service._enqueue_uploader = MagicMock()
        video_recorder_service._uploader_queue = MagicMock()
        video_recorder_service._writer_queue = MagicMock()
        video_recorder_service._is_service_running = False
        video_recorder_service._writer_queue.empty.side_effect = [False, True]
        video_recorder_service._writer_queue.get.return_value = video_recorder_mock
        video_recorder_mock.exceed_max_frames.return_value = False
        video_recorder_mock.should_immediate_stop.return_value = True
        video_recorder_mock.state = VideoRecorderState.STOP
        video_recorder_service._write()
        video_recorder_service._enqueue_uploader.assert_called_once_with(video_recorder_mock)

    def test_write_stop_state_non_immediate_stop_enqueue_writeris_writer_running(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._enqueue_writer = MagicMock()
        video_recorder_service._enqueue_uploader = MagicMock()
        video_recorder_service._uploader_queue = MagicMock()
        video_recorder_service._writer_queue = MagicMock()
        video_recorder_service._enqueue_writer.return_value = True
        video_recorder_service._is_service_running = False
        video_recorder_service._writer_queue.empty.side_effect = [False, True]
        video_recorder_service._writer_queue.get.return_value = video_recorder_mock
        video_recorder_mock.exceed_max_frames.return_value = False
        video_recorder_mock.should_immediate_stop.return_value = False
        video_recorder_mock.state = VideoRecorderState.STOP
        video_recorder_service._write()
        video_recorder_service._enqueue_writer.assert_called_once_with(video_recorder_mock)
        video_recorder_service._enqueue_uploader.assert_not_called()

    def test_write_stop_state_non_immediate_stop_enqueue_uploaderis_writer_running(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._enqueue_writer = MagicMock()
        video_recorder_service._enqueue_uploader = MagicMock()
        video_recorder_service._uploader_queue = MagicMock()
        video_recorder_service._writer_queue = MagicMock()
        video_recorder_service._enqueue_writer.return_value = False
        video_recorder_service._is_service_running = False
        video_recorder_service._writer_queue.empty.side_effect = [False, True]
        video_recorder_service._writer_queue.get.return_value = video_recorder_mock
        video_recorder_mock.exceed_max_frames.return_value = False
        video_recorder_mock.should_immediate_stop.return_value = False
        video_recorder_mock.state = VideoRecorderState.STOP
        video_recorder_service._write()
        video_recorder_service._enqueue_writer.assert_called_once_with(video_recorder_mock)
        video_recorder_service._enqueue_uploader.assert_called_once_with(video_recorder_mock)

    def test_upload_recorder_is_terminated(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._recorders_dict = MagicMock()
        video_recorder_service._uploader_queue = MagicMock()
        video_recorder_service._is_service_running = False
        video_recorder_service._writer_finished_event = MagicMock()
        video_recorder_service._writer_finished_event.is_set.return_value = True
        video_recorder_service._uploader_queue.empty.side_effect = [False, True]
        video_recorder_service._uploader_queue.get.return_value = video_recorder_mock
        video_recorder_mock.is_terminated.return_value = True
        video_recorder_service._upload()
        video_recorder_mock.upload.assert_called_once()
        video_recorder_service._recorders_dict.pop.assert_called_once_with(video_recorder_mock.video_name)

    def test_upload_recorder_is_not_terminated(self, video_recorder_mock, thread_mock, rospy_mock) -> None:
        video_recorder_service = VideoRecorderService()
        video_recorder_service._recorders_dict = MagicMock()
        video_recorder_service._uploader_queue = MagicMock()
        video_recorder_service._is_service_running = False
        video_recorder_service._writer_finished_event = MagicMock()
        video_recorder_service._writer_finished_event.is_set.return_value = True
        video_recorder_service._uploader_queue.empty.side_effect = [False, True]
        video_recorder_service._uploader_queue.get.return_value = video_recorder_mock
        video_recorder_mock.is_terminated.return_value = False
        video_recorder_service._upload()
        video_recorder_mock.upload.assert_called_once()
        video_recorder_service._recorders_dict.pop.assert_not_called()
