cmake_minimum_required(VERSION 2.8.3)
project(kdl_training)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  pr2_controllers_msgs
  roscpp
  tf
  geometry_msgs
  std_msgs
  kdl_parser
  pr2_mechanism_model
  sensor_msgs
  kdl_conversions
 # kdl_msgs
)

find_package(Boost REQUIRED )

find_package(orocos_kdl REQUIRED)

link_directories(
 ${catkin_LIBRARY_DIRS}
 ${orocos_kdl_LIBRARY_DIRS}
)
#find_library(OROCOS_KDL orocos-kdl)
#set(orocos_kdl_LIBRARIES ${OROCOS_KDL})
#find_package(kdl_parser)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#  generate_messages(
#    DEPENDENCIES
#    pr2_controllers_msgs
#    geometry_msgs
#    std_msgs
#  )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

#find_library(orocos_kdl_LIBRARY NAMES ${orocos_kdl_LIBRARIES} PATHS
#${orocos_kdl_LIBRARY_DIRS} NO_DEFAULT_PATH)
#set(orocos_kdl_LIBRARIES ${orocos_kdl_LIBRARY})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES kdl_training
#  CATKIN_DEPENDS actionlib pr2_controllers_msgs roscpp tf
#  DEPENDS system_lib orokos_kdl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/kdl_training.cpp
# )

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_executable(kdl_training_fk src/${PROJECT_NAME}/kdl_training_fk.cpp)
target_link_libraries(kdl_training_fk ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(kdl_training_ik src/${PROJECT_NAME}/kdl_training_ik.cpp)
target_link_libraries(kdl_training_ik ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(kdl_keyboard src/${PROJECT_NAME}/kdl_keyboard.cpp)
target_link_libraries(kdl_keyboard ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(r_arm_controller src/${PROJECT_NAME}/r_arm_controller.cpp)
target_link_libraries(r_arm_controller ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(r_arm_test src/${PROJECT_NAME}/r_arm_test.cpp)
target_link_libraries(r_arm_test ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(r_arm_torso_controller_test src/${PROJECT_NAME}/r_arm_torso_controller_test.cpp)
target_link_libraries(r_arm_torso_controller_test ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(r_arm_movement_controller src/${PROJECT_NAME}/r_arm_movement_controller.cpp)
target_link_libraries(r_arm_movement_controller ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(shapes src/${PROJECT_NAME}/shapes.cpp)
target_link_libraries(shapes ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(kdl_training ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(kdl_training_node src/kdl_training_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(kdl_training_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
  # target_link_libraries(kdl_training
  #   ${catkin_LIBRARIES}
  #   ${orocos_kdl_LIBRARIES}
  # )
  

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS kdl_training kdl_training_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


#############
## Testing ##
#############


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



set(TEST_SRCS
  test/main.cpp
  test/${PROJECT_NAME}/kdl_functions.cpp 
)

catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SRCS}
		 test/main.cpp
		 test/kdl_training/kdl_functions.cpp
		 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test_data)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
endif()
