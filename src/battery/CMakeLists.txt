################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(battery)

set(BMS_QUCC 1)
set(BMS_POWERCRAFT 2)

# 명령줄에서 변수를 전달해서 빌드할 경우($ catkin build battery --cmake-args -DBMS=3)
if(${BMS})
  message("COMMANDLINE BMS")
# 명령줄에서 변수를 전달하지 않고 재컴파일할 경우($ catkin build battery) 이전값이 전달되므로
# 반드시 catkin clean battery를 한 뒤에 빌드할 것
else()
  message("NO commandline BMS")
  set(BMS ${BMS_QUCC})
  # set(BMS ${BMS_POWERCRAFT})
endif()

if(${BMS})
  add_definitions(-DBMS=${BMS})
else()
  add_definitions(-DNOBMS)
endif()

if(${BMS} EQUAL ${BMS_QUCC})
  message("BMS_QUCC BUILD")
elseif(${BMS} EQUAL ${BMS_POWERCRAFT})
  message("BMS_POWERCRAFT BUILD")
else()
  message(FATAL_ERROR "UNKNOWN_BMS BUILD")
endif()
################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
)

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
  LIBRARIES battery
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf
#   DEPENDS system_lib
)

################################################################################
# Build
################################################################################
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS libmodbus REQUIRED)

if(${BMS} EQUAL ${BMS_QUCC})
  include_directories(  
    ${catkin_INCLUDE_DIRS}
  )

  add_executable(battery src/qucc/main.cpp src/qucc/qucc.cpp)
  target_link_libraries(battery ${catkin_LIBRARIES})
  add_dependencies(battery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
elseif(${BMS} EQUAL ${BMS_POWERCRAFT})
  include_directories(  
    ${catkin_INCLUDE_DIRS}
    ${LIBMODBUS_INCLUDE_DIRS}
  )

  add_executable(battery src/powercraft/powercraft.cpp)
  target_link_libraries(battery ${catkin_LIBRARIES} ${LIBMODBUS_LIBRARIES})
  add_dependencies(battery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
else()
endif()

add_executable(dummy_battery src/dummy/dummy.cpp)
target_link_libraries(dummy_battery ${catkin_LIBRARIES} ${LIBMODBUS_LIBRARIES})
add_dependencies(dummy_battery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# I knew the REP0003(http://www.ros.org/reps/rep-0003.html#c),
# but the code of "battery" must compile with C++11.
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


################################################################################
# Install
################################################################################
install(TARGETS battery
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