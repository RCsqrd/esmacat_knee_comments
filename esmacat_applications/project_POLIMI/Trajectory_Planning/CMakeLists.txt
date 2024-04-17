
# Minimum required version of CMake
cmake_minimum_required(VERSION 3.10.2)

# Set the policy CMP0003 to NEW if the cmake_policy command exists
if(COMMAND cmake_policy)
    # CMP0003 policy ensures that full paths are used for libraries
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Define a recursive function to include directories in the project
function(include_sub_directories_recursively root_dir)
    # If the directory exists, include it in the project
    if(IS_DIRECTORY ${root_dir})
        include_directories(${root_dir})
    endif()

    # Get all subdirectories
    file(GLOB ALL_SUB_DIRS RELATIVE ${root_dir} ${root_dir}/*)
    foreach(sub_dir ${ALL_SUB_DIRS})
        # If it's a directory, recursively include it and its subdirectories in the project
        if(IS_DIRECTORY ${root_dir}/${sub_dir})
            include_sub_directories_recursively(${root_dir}/${sub_dir})
        endif()
    endforeach()
endfunction()

# Call the function for each directory you want to include
include_sub_directories_recursively(/home/esmacat/Desktop/esmacat_rt_restore/esmacat_master_high_performance/esmacat_core)
include_sub_directories_recursively(/home/esmacat/Desktop/esmacat_rt_restore/esmacat_master_high_performance/esmacat_applications/sensojoint_master)

include_directories(/home/esmacat/Desktop/esmacat_rt_restore/esmacat_master_high_performance/configuration/hw/esmacat_master_c)
include_directories(/home/esmacat/Desktop/esmacat_rt_restore/esmacat_master_high_performance/SOEM/soem)
include_directories(/home/esmacat/Desktop/esmacat_rt_restore/esmacat_master_high_performance/SOEM/osal)
include_directories(/home/esmacat/Desktop/esmacat_rt_restore/esmacat_master_high_performance/SOEM/osal/linux)
include_directories(/home/esmacat/Desktop/esmacat_rt_restore/esmacat_master_high_performance/SOEM/oshw/linux)

# Define the project name
project(Trajectory_Planning_RESTORE)

file(GLOB THIS_APP_SOURCES sensojoint_master/esmacat_main.cpp sensojoint_master/sensojoint.cpp sensojoint_master/sensojoint_control.cpp sensojoint_master/sensojoint_manager.cpp sensojoint_master/sensojoint_shared_memory.cpp sensojoint_master/sensojoint_adaptive.cpp  )
file(GLOB THIS_APP_HEADERS sensojoint_master/sensojoint.h sensojoint_master/sensojoint_control.h sensojoint_master/sensojoint_manager.h sensojoint_master/sensojoint_shared_memory.h sensojoint_master/sensojoint_structs.h sensojoint_master/sensojoint_adaptive.h )


get_filename_component(THIS_APP_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "Example application, ${THIS_APP_NAME}, is ready to build")

# Add the current source directory to the list of directories to be used to link libraries
link_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_executable(${THIS_APP_NAME} ${THIS_APP_SOURCES} ${THIS_APP_HEADERS})
target_link_libraries(${THIS_APP_NAME} pthread rt esmacat_core esmacat_driver)
set_target_properties(${THIS_APP_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/tmp/run.sh
"sudo ./${THIS_APP_NAME} &
sudo ${PROJECT_BINARY_DIR}/esmacat_core/application/ecat_main")

file (COPY ${CMAKE_CURRENT_BINARY_DIR}/tmp/run.sh DESTINATION ${CMAKE_CURRENT_BINARY_DIR} FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ)
file(REMOVE_RECURSE ${CMAKE_CURRENT_BINARY_DIR}/tmp)


add_executable(${THIS_APP_NAME}_interface sensojoint_master/sensojoint_shared_memory.cpp sensojoint_master/sensojoint_shared_memory.h sensojoint_master/interface_main.cpp sensojoint_master/sensojoint_interface.cpp sensojoint_master/sensojoint_interface.h sensojoint_master/sensojoint_structs.h  )
add_executable(${THIS_APP_NAME}_display sensojoint_master/sensojoint_shared_memory.cpp sensojoint_master/sensojoint_shared_memory.h sensojoint_master/display_main.cpp sensojoint_master/sensojoint_structs.h )
add_executable(${THIS_APP_NAME}_trajectory sensojoint_master/sensojoint_structs.h trajectory_generation.h trajectory_generation.cpp trajectory_generation_main.cpp sensojoint_master/sensojoint_shared_memory.cpp sensojoint_master/sensojoint_shared_memory.h )

target_link_libraries(${THIS_APP_NAME}_trajectory pthread rt)
