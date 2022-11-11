cmake_minimum_required(VERSION 3.5)
project(nlink_parser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to Release build
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()
message( STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}" )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB NLINK_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")

file(GLOB ROS_MSG_DIR_LIST "${MSGS_DIR}/*.msg")

set(ROS_MSG_DIR_LIST "${ROS_MSG_DIR_LIST}" CACHE INTERNAL "ROS_MSG_DIR_LIST")

if(NOT "${NLINK_MSGS}" STREQUAL "")
  # Generate introspection typesupport for C and C++ and IDL files
  rosidl_generate_interfaces(${PROJECT_NAME}
                             ${NLINK_MSGS}
                             DEPENDENCIES
                             std_msgs
                             builtin_interfaces
                             ADD_LINTER_TESTS)
else()
  message(FATAL_ERROR "No Nlink msgs found to generate interfaces to!")
endif()

include_directories(
    include
    src/utils
    )

set(nutils "nutils")
add_library(${nutils}
    src/utils/protocol_extracter/nprotocol_base.cpp
    src/utils/protocol_extracter/nprotocol_extracter.cpp
    src/utils/nlink_unpack/nlink_utils.c
    src/utils/nlink_unpack/nlink_linktrack_anchorframe0.c
    src/utils/nlink_unpack/nlink_linktrack_tagframe0.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe0.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe1.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe2.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe3.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe4.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe5.c
    src/utils/nlink_unpack/nlink_linktrack_nodeframe6.c
    src/utils/nlink_unpack/nlink_tofsense_frame0.c
    src/utils/nlink_unpack/nlink_tofsensem_frame0.c
    src/utils/nlink_unpack/nlink_linktrack_aoa_nodeframe0.c
    src/utils/nlink_unpack/nlink_iot_frame0.c
    src/utils/init_serial.cpp
    src/utils/nlink_protocol.cpp
    src/utils/nutils.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${nutils} "${cpp_typesupport_target}") 
ament_target_dependencies(${nutils} rclcpp serial std_msgs nav_msgs geometry_msgs)

set(LINKTRACK_NAME "linktrack")
add_executable(${LINKTRACK_NAME}
    src/linktrack/init.cpp
    src/linktrack/main.cpp
    src/linktrack/protocols.cpp
    )
target_link_libraries(${LINKTRACK_NAME} ${nutils})
install(TARGETS
${LINKTRACK_NAME}
DESTINATION lib/${PROJECT_NAME})

set(LINKTRACK_RVIZ_NAME "linktrack_rviz_converter")
add_executable(${LINKTRACK_RVIZ_NAME}
    src/linktrack/main_rviz_converter.cpp
    )
target_link_libraries(${LINKTRACK_RVIZ_NAME}
    ${nutils}
    )
install(TARGETS
    ${LINKTRACK_RVIZ_NAME}
    DESTINATION lib/${PROJECT_NAME})

set(TOFSENSE_NAME "tofsense")
add_executable(${TOFSENSE_NAME}
    src/tofsense/init.cpp
    src/tofsense/main.cpp
    )
target_link_libraries(${TOFSENSE_NAME}
    ${nutils}
    )
install(TARGETS
${EXE_NAME}
DESTINATION lib/${PROJECT_NAME})

set(EXE_NAME "tofsensem")
add_executable(${EXE_NAME}
    src/tofsensem/init.cpp
    src/tofsensem/main.cpp
    )
target_link_libraries(${EXE_NAME}
    ${nutils}
    )
install(TARGETS
${EXE_NAME}
DESTINATION lib/${PROJECT_NAME})

set(EXE_NAME "linktrack_aoa")
add_executable(${EXE_NAME}
    src/linktrack_aoa/init.cpp
    src/linktrack_aoa/main.cpp
    src/linktrack/protocols.cpp
    )
target_link_libraries(${EXE_NAME} ${nutils})
install(TARGETS
    ${EXE_NAME}
    DESTINATION lib/${PROJECT_NAME})

set(EXE_NAME "iot")
add_executable(${EXE_NAME}
    src/iot/init.cpp
    src/iot/main.cpp
    )
target_link_libraries(${EXE_NAME}
    ${nutils}
    )
install(TARGETS
    ${EXE_NAME}
    DESTINATION lib/${PROJECT_NAME})

#############
## Testing ##
#############
if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}_test
        src/linktrack/init.cpp
        src/linktrack/protocols.cpp
        src/tofsense/init.cpp
        src/linktrack_aoa/init.cpp
        test/test_nlink_parser.cpp
        src/tofsensem/init.cpp
        src/iot/init.cpp
    )
endif()

if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test  ${nutils})
endif()

ament_package()
