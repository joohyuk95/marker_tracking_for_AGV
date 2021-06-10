cmake_minimum_required(VERSION 3.0.2)
project(agv_autorace)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(

)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY cfg file launch param
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
