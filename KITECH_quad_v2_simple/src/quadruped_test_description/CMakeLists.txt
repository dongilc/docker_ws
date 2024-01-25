cmake_minimum_required(VERSION 2.8.3)
project(quadruped_test_description)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  joy
)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Boost REQUIRED)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS joy roscpp std_msgs sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  SYSTEM 
  ${EIGEN3_INCLUDE_DIR}  
  ${catkin_INCLUDE_DIRS}
)

# add_executable(mpc_test control/cpp/dm_mpc.cpp)
# add_dependencies(mpc_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(mpc_test ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)

add_executable(mpc_test_xps control/cpp/dm_mpc_quad2.cpp)
add_dependencies(mpc_test_xps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_test_xps ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)

add_executable(mpc_test_sip control/cpp/dm_mpc_quad2_sip.cpp)
add_dependencies(mpc_test_sip ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_test_sip ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)