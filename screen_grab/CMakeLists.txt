cmake_minimum_required(VERSION 3.0.2)
project(screen_grab)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge  # image_show
  dynamic_reconfigure
  image_transport
  nodelet
  roscpp
  # roslint
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED COMPONENTS highgui)

# roslint_cpp()

generate_dynamic_reconfigure_options(
  cfg/ScreenGrab.cfg
)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS dynamic_reconfigure nodelet roscpp sensor_msgs std_msgs
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}.cpp
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   X11
)
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

add_library(image_show
   src/image_show.cpp
)

target_link_libraries(image_show
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

install(TARGETS
  image_show
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  nodelet_image_show.xml
  nodelet_${PROJECT_NAME}.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
