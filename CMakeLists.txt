cmake_minimum_required(VERSION 2.8.3)
project(agilex_scout)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs tf
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs tf nav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(agilex_scout src/agilex_scout_node.cpp)
target_link_libraries(agilex_scout ${catkin_LIBRARIES} )

install(TARGETS agilex_scout
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
     )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
