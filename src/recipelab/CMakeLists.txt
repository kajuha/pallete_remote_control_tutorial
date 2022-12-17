################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(recipelab)

# $ rosrun rqt_reconfigure rqt_reconfigure
set(GAIN_TUNING TRUE)
# set(GAIN_TUNING FALSE)
if(${GAIN_TUNING})
  add_definitions(-DGAIN_TUNING)
endif()

################################################################################
# Packages
################################################################################
if(${GAIN_TUNING})
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  chatterbox
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/GainTuning.cfg
)
else()
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  chatterbox
  # arduino_safety
)
endif()

# add_dependencies(recipelab_gain recipelab_gain_gencfg)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(  
  LIBRARIES recipelab
  CATKIN_DEPENDS roscpp std_msgs
  # DEPENDS system_lib
)

################################################################################
# Build
################################################################################
include_directories(  
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)

add_executable(recipelab src/main.cpp src/AGVCallBack.cpp src/AGVSource.cpp)
add_dependencies(recipelab ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(recipelab ${catkin_LIBRARIES})

# I knew the REP0003(http://www.ros.org/reps/rep-0003.html#c),
# but the code of "recipelab" must compile with C++11.
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


################################################################################
# Install
################################################################################
install(TARGETS recipelab
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
#############################################################################