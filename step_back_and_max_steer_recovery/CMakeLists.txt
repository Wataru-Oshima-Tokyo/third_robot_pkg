cmake_minimum_required(VERSION 3.0.5)
project(step_back_and_max_steer_recovery)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            #tf
            tf2
            tf2_ros
            tf2_eigen
            tf2_geometry_msgs
            costmap_2d
            nav_core
            pluginlib
            base_local_planner
        )
find_package(Eigen3)

find_package(Eigen REQUIRED)
if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(Eigen3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_DEFINITIONS ${EIGEN_DEFINITIONS})
endif()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    CATKIN_DEPENDS
        roscpp
        pluginlib
)

add_library(${PROJECT_NAME} src/step_back_and_max_steer_recovery.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES move_slow_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
