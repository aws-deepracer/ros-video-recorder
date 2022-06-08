cmake_minimum_required(VERSION 3.7.2)
project(video_recorder_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
)

add_service_files(DIRECTORY srv FILES
  VideoRecorderPauseSrv.srv
  VideoRecorderStartSrv.srv
  VideoRecorderStopSrv.srv
  VideoRecorderUnpauseSrv.srv
  VideoRecorderChangeSrv.srv
  )

generate_messages(DEPENDENCIES
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  )
