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
"""A class for Video Recorder Service."""
import os
import rospy
import queue
import numpy as np

from threading import RLock, Thread, Event
from video_recorder.constants import (DEFAULT_VIDEO_FRAME_WIDTH,
                                      DEFAULT_VIDEO_FRAME_HEIGHT,
                                      DEFAULT_VIDEO_FOURCC,
                                      DEFAULT_VIDEO_FPS,
                                      DEFAULT_FORMAT,
                                      VideoRecorderServiceTopic,
                                      VideoRecorderState)
from video_recorder_msgs.srv import (
    VideoRecorderStartSrv,
    VideoRecorderStartSrvRequest, VideoRecorderStartSrvResponse,
    VideoRecorderPauseSrv,
    VideoRecorderPauseSrvRequest, VideoRecorderPauseSrvResponse,
    VideoRecorderUnpauseSrv,
    VideoRecorderUnpauseSrvRequest, VideoRecorderUnpauseSrvResponse,
    VideoRecorderStopSrv,
    VideoRecorderStopSrvRequest, VideoRecorderStopSrvResponse,
    VideoRecorderChangeSrv, VideoRecorderChangeSrvResponse,
    VideoRecorderChangeSrvRequest)
from video_recorder.video_recorder import VideoRecorder
from video_recorder.video_writer import VideoWriter
from video_recorder.video_uploader import VideoUploader
from video_recorder.video_subscriber import (VideoSubscriber,
                                             VideoSubscriberObserverInterface)
from video_recorder.set_queue import SetQueue


class VideoRecorderService(VideoSubscriberObserverInterface):
    """
    VideoRecorderService class
    """

    def __init__(self) -> None:
        """
        VideoRecorderService constructor
        """
        self._lock = RLock()
        self._recorders_dict = dict()
        self._uploader_queue = queue.Queue()
        self._writer_queue = SetQueue()
        self._is_service_running = False
        self._writer_finished_event = Event()

    def on_frame_received(self, video_subscriber: VideoSubscriber, frame: np.ndarray) -> None:
        """
        VideoSubscriberObserverInterface callback when frame is received

        Args:
            video_subscriber (VideoSubscriber): video_subscriber
            frame (np.ndarray): image frame ndarray
        """
        with self._lock:
            video_name = video_subscriber.video_name
            if video_name in self._recorders_dict:
                self._writer_queue.put(self._recorders_dict[video_name])

    def start(self, timeout: float = 1.0) -> None:
        """
        Start the video recorder service by starting ROS service
        and start both writer and uploader thread

        Args:
            timeout (float): timeout in seconds for both uploader
                             and writer queue.
        """
        with self._lock:
            if not self._is_service_running:
                self._uploader_thread = Thread(target=self._upload, args=(timeout,))
                self._writer_thread = Thread(target=self._write, args=(timeout,))
                rospy.loginfo("[VideoRecorderService]: Service starting...")
                self._video_recorder_start_srv = rospy.Service(
                    VideoRecorderServiceTopic.START,
                    VideoRecorderStartSrv,
                    self._start_callback)
                self._video_recorder_pause_srv = rospy.Service(
                    VideoRecorderServiceTopic.PAUSE,
                    VideoRecorderPauseSrv,
                    self._pause_callback)
                self._video_recorder_unpause_srv = rospy.Service(
                    VideoRecorderServiceTopic.UNPAUSE,
                    VideoRecorderUnpauseSrv,
                    self._unpause_callback)
                self._video_recorder_stop_srv = rospy.Service(
                    VideoRecorderServiceTopic.STOP,
                    VideoRecorderStopSrv,
                    self._stop_callback)
                self._video_recorder_change_srv = rospy.Service(
                    VideoRecorderServiceTopic.CHANGE,
                    VideoRecorderChangeSrv,
                    self._change_callback)
                self._is_service_running = True
                self._writer_thread.start()
                self._uploader_thread.start()
                rospy.loginfo("[VideoRecorderService]: Service started")
            else:
                rospy.loginfo("[VideoRecorderService]: Service has already started "
                              "and cannot start again without stopping first")

    def stop(self) -> None:
        """
        Stop the video recorder service by shutdown ROS service
        """
        with self._lock:
            if self._is_service_running:
                rospy.loginfo("[VideoRecorderService]: Service stopping...")
                self._video_recorder_start_srv.shutdown()
                self._video_recorder_pause_srv.shutdown()
                self._video_recorder_unpause_srv.shutdown()
                self._video_recorder_stop_srv.shutdown()
                self._video_recorder_change_srv.shutdown()
                # immediate stop all writer before stop the service
                for _, video_recorder in self._recorders_dict.items():
                    if video_recorder.state != VideoRecorderState.STOP:
                        video_recorder.stop(should_immediate_stop=True)
                with self._lock:
                    self._is_service_running = False
                self._writer_thread.join()
                self._uploader_thread.join()
                rospy.loginfo("[VideoRecorderService]: Service stopped")
            else:
                rospy.loginfo("[VideoRecorderService]: Service has already stopped "
                              "and cannot stop again without starting first")

    def _start_callback(self,
                        request: VideoRecorderStartSrvRequest) -> VideoRecorderStartSrvResponse:
        """
        Callback handler for start request

        Args:
            request (VideoRecorderStartSrvRequest): request containing start values

        Returns:
            VideoRecorderStartSrvResponse: response
        """
        response = VideoRecorderStartSrvResponse()
        response.success = False
        with self._lock:
            if request.video_name in self._recorders_dict:
                message = "[VideoRecorderService] video recorder {} " \
                          "failed to start: video name in use, please enter a " \
                          "different video name".format(request.video_name)
            else:
                try:
                    os.makedirs(request.local_dir, exist_ok=True)
                    self._recorders_dict[request.video_name] = VideoRecorder(
                        video_name=request.video_name,
                        writer=VideoWriter(
                            video_name=request.video_name,
                            local_dir=request.local_dir,
                            video_format=request.video_format if request.video_format else DEFAULT_FORMAT,
                            max_num_frames=request.max_num_frames if request.max_num_frames > 0 else float("inf"),
                            fourcc=request.fourcc if request.fourcc > 0 else DEFAULT_VIDEO_FOURCC,
                            fps=request.fps if request.fps > 0 else DEFAULT_VIDEO_FPS,
                            frame_width=request.frame_width if request.frame_width > 0 else DEFAULT_VIDEO_FRAME_WIDTH,
                            frame_height=request.frame_height if request.frame_height > 0 else DEFAULT_VIDEO_FRAME_HEIGHT),
                        uploader=VideoUploader(
                            video_name=request.video_name,
                            local_dir=request.local_dir,
                            video_format=request.video_format if request.video_format else DEFAULT_FORMAT,
                            s3_bucket=request.s3_bucket,
                            s3_prefix=request.s3_prefix,
                            s3_region=request.s3_region),
                        subscriber=VideoSubscriber(
                            video_name=request.video_name,
                            video_topic=request.video_topic,
                            observer=self))
                    response.success = True
                    message = "[VideoRecorderService]: video recorder {} " \
                              "start successfully".format(request.video_name)
                except Exception as ex:
                    message = "[VideoRecorderService]: video recorder {} " \
                              "failed to start: {}".format(request.video_name, ex)
        response.message = message
        rospy.loginfo(message)
        return response

    def _pause_callback(self,
                        request: VideoRecorderPauseSrvRequest) -> VideoRecorderPauseSrvResponse:
        """
        Callback handler for pause request

        Args:
            request (VideoRecorderPauseSrvRequest): request containing pause values

        Returns:
            VideoRecorderPauseSrvResponse: response
        """
        response = VideoRecorderPauseSrvResponse()
        response.success = False
        with self._lock:
            if request.video_name not in self._recorders_dict:
                message = "[VideoRecorderService] video recorder {} " \
                          "failed to pause: video name does not exit, please " \
                          "start first".format(request.video_name)
            else:
                try:
                    self._recorders_dict[request.video_name].pause()
                    response.success = True
                    message = "[VideoRecorderService]: video recorder {} " \
                              "pause successfully".format(request.video_name)
                except Exception as ex:
                    message = "[VideoRecorderService]: video recorder {} " \
                              "failed to pause: {}".format(request.video_name, ex)
        response.message = message
        rospy.loginfo(message)
        return response

    def _unpause_callback(self,
                          request: VideoRecorderUnpauseSrvRequest) \
            -> VideoRecorderUnpauseSrvResponse:
        """
        Callback handler for unpause request

        Args:
            request (VideoRecorderUnpauseSrvRequest): request containing unpause values

        Returns:
            VideoRecorderUnpauseSrvResponse: response
        """
        response = VideoRecorderUnpauseSrvResponse()
        response.success = False
        with self._lock:
            if request.video_name not in self._recorders_dict:
                message = "[VideoRecorderService] video recorder {} " \
                          "failed to unpause: video name does not exit, please " \
                          "start first".format(request.video_name)
            else:
                try:
                    self._recorders_dict[request.video_name].unpause()
                    response.success = True
                    message = "[VideoRecorderService]: video recorder {} " \
                              "unpause successfully".format(request.video_name)
                except Exception as ex:
                    message = "[VideoRecorderService]: video recorder {} " \
                              "failed to unpause: {}".format(request.video_name, ex)
        response.message = message
        rospy.loginfo(message)
        return response

    def _change_callback(self,
                         request: VideoRecorderChangeSrvRequest) -> VideoRecorderChangeSrvResponse:
        """
        Callback handler for change request

        Args:
            request (VideoRecorderChangeSrvRequest): request containing change values

        Returns:
            VideoRecorderChangeSrvResponse: response
        """
        response = VideoRecorderChangeSrvResponse()
        response.success = False
        with self._lock:
            if request.video_name not in self._recorders_dict:
                message = "[VideoRecorderService] video recorder {} " \
                          "failed to change: video name does not exit, please " \
                          "start first".format(request.video_name)
            else:
                try:
                    self._recorders_dict[request.video_name].change(request.video_topic)
                    response.success = True
                    message = "[VideoRecorderService]: video recorder {} " \
                              "change successfully".format(request.video_name)
                except Exception as ex:
                    message = "[VideoRecorderService]: video recorder {} " \
                              "failed to change: {}".format(request.video_name, ex)
        response.message = message
        rospy.loginfo(message)
        return response

    def _stop_callback(self, request: VideoRecorderStopSrvRequest) -> VideoRecorderStopSrvResponse:
        """
        Callback handler for stop request

        Args:
            request (VideoRecorderStopSrvRequest): request containing stop values

        Returns:
            VideoRecorderStopSrvResponse: response
        """
        response = VideoRecorderStopSrvResponse()
        response.success = False
        with self._lock:
            if request.video_name not in self._recorders_dict:
                message = "[VideoRecorderService] video recorder {} " \
                          "failed to stop: video name does not exit, please " \
                          "start first".format(request.video_name)
            else:
                try:
                    self._recorders_dict[request.video_name].stop(request.should_immediate_stop)
                    response.success = True
                    message = "[VideoRecorderService]: video recorder {} " \
                              "stop successfully".format(request.video_name)
                except Exception as ex:
                    message = "[VideoRecorderService]: video recorder {} " \
                              "failed to stop: {}".format(request.video_name, ex)
        response.message = message
        rospy.loginfo(message)
        return response

    def _enqueue_writer(self, video_recorder) -> bool:
        """
        Enqueue VideoRecorder instance into writer queue

        Args:
            video_recorder (VideoRecorder): VideoRecorder class instance

        Returns:
            bool: True if successfully enqueue and False otherwise
        """
        with self._lock:
            if not video_recorder.is_empty():
                self._writer_queue.put(video_recorder)
                return True
            return False

    def _enqueue_uploader(self, video_recorder) -> None:
        """
        Enqueue VideoRecorder instance into uploader queue

        Args:
            video_recorder (VideoRecorder): VideoRecorder class instance
        """
        video_recorder.reset()
        self._uploader_queue.put(video_recorder)
        rospy.loginfo("[VideoRecorderService] {} enqueue uploader".format(
            video_recorder.video_name))

    def _write(self, timeout: float = 1.0) -> None:
        """
        Write to local file executed in writer thread

        Args:
            timeout (float): timeout in seconds for writer queue get
        """
        while self._is_service_running or not self._writer_queue.empty():
            try:
                video_recorder = self._writer_queue.get(timeout=timeout)
                video_recorder.write()
            except queue.Empty as ex:
                rospy.loginfo("[VideoRecorderService]: writer thread timeout "
                              "after {} seconds: {}".format(timeout, ex))
            else:
                if video_recorder.exceed_max_frames():
                    self._enqueue_uploader(video_recorder)
                if video_recorder.state in [VideoRecorderState.RUN, VideoRecorderState.PAUSE]:
                    self._enqueue_writer(video_recorder)
                elif video_recorder.state == VideoRecorderState.STOP:
                    if video_recorder.should_immediate_stop():
                        self._enqueue_uploader(video_recorder)
                    else:
                        success = self._enqueue_writer(video_recorder)
                        if not success:
                            self._enqueue_uploader(video_recorder)
        self._writer_finished_event.set()
        rospy.loginfo("[VideoRecorderService]: writer thread stop to join")

    def _upload(self, timeout: float = 1.0) -> None:
        """
        Upload to s3 executed in uploader thread

        Args:
            timeout (float): timeout in seconds for uploader queue get
        """
        while self._is_service_running or \
                not self._writer_finished_event.is_set() or \
                not self._uploader_queue.empty():
            try:
                video_recorder = self._uploader_queue.get(timeout=timeout)
            except queue.Empty as ex:
                rospy.loginfo("[VideoRecorderService]: uploader thread timeout "
                              "after {} seconds: {}".format(timeout, ex))
            else:
                video_recorder.upload()
                if video_recorder.is_terminated():
                    with self._lock:
                        self._recorders_dict.pop(video_recorder.video_name)
        rospy.loginfo("[VideoRecorderService]: uploader thread stop to join")
