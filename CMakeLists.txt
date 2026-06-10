cmake_minimum_required(VERSION 3.0.2)
project(ars620_driver)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roslib
  roscpp
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  Ars620ConfigState.msg
  Ars620SystemStatus.msg
  Ars620RdiTarget.msg
  Ars620RdiTargetArray.msg
  Ars620OdTarget.msg
  Ars620OdTargetArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roslib roscpp sensor_msgs std_msgs
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(CONTROLCANFD_ARCH "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(CONTROLCANFD_ARCH "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|armv7|armv7l)$")
  set(CONTROLCANFD_ARCH "aarch32")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^riscv64$")
  set(CONTROLCANFD_ARCH "riscv64")
else()
  message(FATAL_ERROR "Unsupported controlcanfd architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
set(CONTROLCANFD_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/lib/libcontrolcanfd/${CONTROLCANFD_ARCH})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CONTROLCANFD_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_core
  src/ars620_decoder.cpp
  src/canfd_frame_logger.cpp
  src/cycle_assembler.cpp
  src/pointcloud.cpp
  src/processing_timing.cpp
  src/usbcanfd_receiver.cpp
)
add_dependencies(${PROJECT_NAME}_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_core ${catkin_LIBRARIES} dl)

add_executable(ars620_driver_node src/ars620_driver_node.cpp)
add_dependencies(ars620_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ars620_driver_node ${PROJECT_NAME}_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_raw_canfd_csv_to_mf4.py)

  catkin_add_gtest(test_ars620_driver
    test/test_ars620_decoder.cpp
    test/test_canfd_frame_logger.cpp
    test/test_processing_timing.cpp
    test/test_vendor_receiver.cpp
  )
  if(TARGET test_ars620_driver)
    target_link_libraries(test_ars620_driver ${PROJECT_NAME}_core ${catkin_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
  endif()

  add_library(fake_controlcanfd SHARED test/fake_controlcanfd.cpp)
  target_include_directories(fake_controlcanfd PRIVATE ${CONTROLCANFD_INCLUDE_DIR})
  if(TARGET test_ars620_driver)
    add_dependencies(test_ars620_driver fake_controlcanfd)
    target_compile_definitions(test_ars620_driver PRIVATE
      FAKE_CONTROLCANFD_PATH="$<TARGET_FILE:fake_controlcanfd>")
  endif()
endif()

install(TARGETS ${PROJECT_NAME}_core ars620_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS scripts/raw_canfd_csv_to_mf4.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(DIRECTORY lib/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lib
)
