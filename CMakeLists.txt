cmake_minimum_required(VERSION 2.8.3)
project(bluesea2)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  sensor_msgs
  message_generation
)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++0x -Wfatal-errors")

add_service_files(
   FILES
   Control.srv
 )

generate_messages(
   DEPENDENCIES
   geometry_msgs
 )

catkin_package(
CATKIN_DEPENDS message_runtime
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(BLUESEA_SDK_PATH "./sdk/")
FILE(GLOB BLUESEA_SDK_SRC
   "${BLUESEA_SDK_PATH}/src/*.cpp",
   "${BLUESEA_SDK_PATH}/src/*.c"
)

add_executable(${PROJECT_NAME}_node  ./src/node.cpp ${BLUESEA_SDK_SRC})
add_executable(${PROJECT_NAME}_client ./src/client.cpp)
add_executable(${PROJECT_NAME}_heart_check ./src/heart_check.cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg  bluesea2_generate_messages_cpp   ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_client ${PROJECT_NAME}_gencfg bluesea2_generate_messages_cpp   ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_heart_check ${PROJECT_NAME}_gencfg bluesea2_generate_messages_cpp   ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_client ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_heart_check ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_client ${PROJECT_NAME}_heart_check
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
