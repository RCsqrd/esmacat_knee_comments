cmake_minimum_required(VERSION 3.5)

project(esmacat_high_performance LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ROS_ENABLE FALSE)

if (ROS_ENABLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic")

    ##########################################
    ##    FIND REQUIRED CATKIN PACKAGES     ##
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      message_generation
      roslaunch
    )
    ##########################################

    ##########################################
    ##     FIND REQUIRED BOOST PACKAGES     ##
    find_package(
      Boost REQUIRED COMPONENTS
      system
      thread
    )
    ##########################################

    # Add the EASE_ROS subfolder to include message files
    file(GLOB ESMACAT_ROS_HARDWARE_FOLDERS ROS/ethercat_arduino_shield_by_esmacat_ROS)

    ##################################################################################
    ##                          ADD ROS MESSAGE FILES                               ##
    ## Add location of message files for the package recursively
    foreach(subdir ${ESMACAT_ROS_HARDWARE_FOLDERS})
      add_message_files(
        # Directory name is sufficient as it looks for all .msg files in the folder
        # It is not required to explicitly include each message file
        DIRECTORY
        ${subdir}/msg
      )
    endforeach()
    ##################################################################################

    ## Generate added messages
    generate_messages(
      DEPENDENCIES
      std_msgs
    )

    ##################################################################################
    ##                           ADD ROS LAUNCH FILES                               ##
    # Add all subfolders within the esmacat_application folder
    file(GLOB ROS_APPLICATION_FOLDERS ROS/ros_esmacat_applications/*)

    ## Add location of launch files for the package
    foreach(subdir ${ROS_APPLICATION_FOLDERS})
      # message("subdir in app is ${subdir}")
      if(EXISTS ${subdir}/launch)
        roslaunch_add_file_check(${subdir}/launch)
      elseif(EXISTS ${subdir}/../launch)
        roslaunch_add_file_check(${subdir}/../launch)
      endif()
    endforeach()
    ##################################################################################

    #########################################################
    ##          CATKIN SPECIFIC CONFIGURATION              ##
    catkin_package(
     CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
     DEPENDS Boost
    )
    #########################################################
endif()

message("Starting Esmacat cmake")
message("Finding system configuration")
message("-- CMAKE_SYSTEM_NAME:      ${CMAKE_SYSTEM_NAME}")
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message("-- CMAKE_SYSTEM:           ${CMAKE_SYSTEM}")

string(FIND ${CMAKE_SYSTEM} "esmacat-master-s" esmacat_master_hw_type_s)
if ( NOT (${esmacat_master_hw_type_s} STREQUAL "-1"))
    message ("Esmacat hardware is Esmacat Master S")
    include_directories(configuration/hw/esmacat_master_s/)
endif()
string(FIND ${CMAKE_SYSTEM} "rockchip" esmacat_master_hw_type_c)
if ( NOT (${esmacat_master_hw_type_c} STREQUAL "-1"))
    message ("Esmacat hardware is Esmacat Master C with TinkerBoard")
    include_directories(configuration/include/hw/esmacat_master_c/)
endif()

string(FIND ${CMAKE_SYSTEM} "4.19.71-rt24-v7l+" esmacat_master_hw_type_c)
if ( NOT (${esmacat_master_hw_type_c} STREQUAL "-1"))
    message ("Esmacat hardware is Esmacat Master C with Rpi4")
    include_directories(configuration/hw/esmacat_master_c/)
endif()
include_directories(configuration/hw/esmacat_master_s/)

############################################################################
##                  INCLUDE NECESSARY DIRECTORIES                         ##
if(ROS_ENABLE)
    include_directories(
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
    )
endif()

include_directories(esmacat_core/include)
include_directories(esmacat_slave_drivers/drivers)
include_directories(resource)
include_directories(configuration)
include_directories(SOEM/soem)
include_directories(SOEM/osal)
include_directories(SOEM/osal/linux)
include_directories(SOEM/oshw/linux)
include_directories(resource/eigen_library)
include_directories(resource/file_handling/include/json/single_include/nlohmann)
include_directories(resource/file_handling/include)

# Recursively add include folders for each Esmacat hardware ROS Library
if(ROS_ENABLE)
    foreach(subdir ${ESMACAT_ROS_HARDWARE_FOLDERS})
      if(EXISTS ${subdir}/include)
        include_directories(${subdir}/include)
      endif()
    endforeach()
endif()
############################################################################

## create esmacat core library and compile it.
message(STATUS "esmacat_core library creating")
file(GLOB ESMACAT_CORE_SOURCES esmacat_core/src/*.cpp)
add_library(esmacat_core STATIC
    ${ESMACAT_CORE_SOURCES}
    )
target_link_libraries(esmacat_core pthread rt)

## create esmacat driver library and compile it.
message(STATUS "esmacat_driver library creating")
file(GLOB ESMACAT_DRIVER_SOURCES esmacat_slave_drivers/drivers/*.cpp)
add_library(esmacat_driver STATIC
    ${ESMACAT_DRIVER_SOURCES}
    )
target_link_libraries(esmacat_driver file_handling esmacat_core)

message(STATUS "file handling library creating")
file(GLOB FILE_HANDLING_SOURCES resource/file_handling/src/*.cpp)
add_library(file_handling STATIC
    ${FILE_HANDLING_SOURCES}
    )

## create SOEM library and compile it.
file(GLOB SOEM_SOURCES SOEM/soem/*.c)
file(GLOB OSAL_SOURCES SOEM/osal/linux/*.c)
file(GLOB OSHW_SOURCES SOEM/oshw/linux/*.c)
add_library(ethercat_driver STATIC
    ${SOEM_SOURCES}
    ${OSAL_SOURCES}
    ${OSHW_SOURCES}
    )
target_link_libraries(ethercat_driver pthread)

# Variables to store ROS Files
file(GLOB_RECURSE esmacat_ros_HEADERS ROS/ethercat_arduino_shield_by_esmacat_ROS/*.h)
file(GLOB_RECURSE esmacat_ros_SOURCES ROS/ethercat_arduino_shield_by_esmacat_ROS/*.cpp)

link_directories(${PROJECT_BINARY_DIR})

##########################################################################
##                 INSTALL FILES IN DESIRED LOCATIONS                   ##
# Install targets in the Project binary directory
install(TARGETS esmacat_core DESTINATION ${PROJECT_BINARY_DIR})
install(TARGETS ethercat_driver DESTINATION ${PROJECT_BINARY_DIR})
install(TARGETS esmacat_driver DESTINATION ${PROJECT_BINARY_DIR})

# Install ROS Headers in the Project binary directory
if(ROS_ENABLE)
    install(FILES ${esmacat_ros_HEADERS}
      DESTINATION ${PROJECT_BINARY_DIR}
    )
    add_subdirectory(ROS)
endif()
##########################################################################

add_subdirectory(esmacat_applications)
add_subdirectory(esmacat_core/application)
add_subdirectory(esmacat_slave_drivers)

