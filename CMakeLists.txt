cmake_minimum_required(VERSION 2.8.3)
project(asv_sim)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  geographic_msgs
  marine_msgs
  rosgraph_msgs
  rospy
  dynamic_reconfigure
  message_generation
)

catkin_python_setup()

add_service_files(DIRECTORY srv
        FILES
        SetPose.srv
)

generate_messages(
        DEPENDENCIES
        geographic_msgs
        marine_msgs
)

generate_dynamic_reconfigure_options(
    cfg/asv_sim.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES asv_sim
  CATKIN_DEPENDS message_runtime geographic_msgs marine_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#add_dependencies(asv_sim_node.py ${PROJECT_NAME}_gencfg)

catkin_install_python(PROGRAMS nodes/asv_sim_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
