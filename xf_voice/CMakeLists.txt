cmake_minimum_required(VERSION 2.8.3)
project(xf_voice)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xf_voice
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(xf_tts src/xf_tts.cpp)
target_link_libraries(xf_tts ${catkin_LIBRARIES} -lmsc  -ldl -lpthread -lm -lrt)
add_dependencies(xf_tts xf_voice_generate_messages_cpp)

add_executable(xf_asr src/xf_asr.cpp)
target_link_libraries(xf_asr ${catkin_LIBRARIES} /Robot/voice/lib/libmsc.so  -ldl -lpthread -lm -lrt -lasound)
add_dependencies(xf_asr xf_voice_generate_messages_cpp)

add_executable(wakeup src/wakeup.cpp)
target_link_libraries(wakeup ${catkin_LIBRARIES}   -lmraa)
add_dependencies(wakeup xf_voice_generate_messages_cpp)
