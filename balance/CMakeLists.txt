cmake_minimum_required(VERSION 2.8.3)
project(balance)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  nav_msgs
)

add_message_files(
  FILES
  Num.msg
  carOdom.msg
 )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )


 generate_messages()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES balance
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}

)


add_executable(balance src/balance.cpp)
add_dependencies(balance balance_cpp})
 target_link_libraries(balance   ${catkin_LIBRARIES} -lmraa )

add_executable(arduino src/arduino.cpp)
add_dependencies(arduino arduino})
 target_link_libraries(arduino   ${catkin_LIBRARIES} -lmraa )


add_executable(odom_pub src/odom_pub.cpp)
target_link_libraries(odom_pub ${catkin_LIBRARIES})
add_dependencies(odom_pub balance_cpp)
