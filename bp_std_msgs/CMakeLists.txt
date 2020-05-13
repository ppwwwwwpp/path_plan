cmake_minimum_required(VERSION 2.8.3)
project(bp_std_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
	std_msgs
        geometry_msgs
        nav_msgs
        ) 


#message
add_message_files(
    FILES
    PathTask.msg
    ActionEntry.msg
    Actions.msg
    GridPoint.msg
    LineEntry.msg
    Lines.msg
    PointEntry.msg
    Points.msg
    RotationData.msg
    GraphPath.msg
    PathEntry.msg
    PathGroupEntry.msg
    PathGroups.msg
    Paths.msg
    Pose2D.msg
    StartParam.msg
    TaskEntry.msg
    TaskData.msg
    AlongWallBow.msg
    AlongWallBowTask.msg
    StartAlongWallOrBow.msg
    TaskOption.msg
    StartSimplePointNav.msg
  TaskDataList.msg
    StartAlongWall.msg
  VirtualWall.msg
PathArea.msg
#PathAreaList.msg

)


#message dependence
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs message_runtime
#  DEPENDS system_lib

)


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   PATTERN ".svn" EXCLUDE
# )
# install(DIRECTORY include
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h*")
