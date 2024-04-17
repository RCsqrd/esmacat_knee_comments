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

include_sub_directories_recursively(${CMAKE_CURRENT_SOURCE_DIR}/stimulator_libraries)
include_sub_directories_recursively(/home/esmacat/Desktop/esmacat_rt_restore/esmacat_master_high_performance/esmacat_core)


include_directories(/home/esmacat/Desktop/esmacat_rt_restore/esmacat_master_high_performance/configuration/hw/esmacat_master_c)
include_directories(/home/esmacat/Desktop/esmacat_rt_restore/esmacat_master_high_performance/SOEM/soem)
include_directories(/home/esmacat/Desktop/esmacat_rt_restore/esmacat_master_high_performance/SOEM/osal)
include_directories(/home/esmacat/Desktop/esmacat_rt_restore/esmacat_master_high_performance/SOEM/osal/linux)
include_directories(/home/esmacat/Desktop/esmacat_rt_restore/esmacat_master_high_performance/SOEM/oshw/linux)

project(sensojoint_master_RESTORE)

file(COPY /home/esmacat/esmacat_rt/esmacat_master_high_performance/esmacat_applications/Trajectory_Planning/trajectory_generation.h DESTINATION /home/esmacat/esmacat_rt/esmacat_master_high_performance/esmacat_applications/sensojoint_master)
file(COPY /home/esmacat/esmacat_rt/esmacat_master_high_performance/esmacat_applications/Trajectory_Planning/trajectory_generation.cpp DESTINATION /home/esmacat/esmacat_rt/esmacat_master_high_performance/esmacat_applications/sensojoint_master)
file(GLOB THIS_APP_SOURCES esmacat_main.cpp sensojoint.cpp sensojoint_control.cpp sensojoint_manager.cpp sensojoint_shared_memory.cpp )
file(GLOB THIS_APP_HEADERS sensojoint.h sensojoint_control.h sensojoint_control.h sensojoint_manager.h sensojoint_shared_memory.h sensojoint_structs.h )

get_filename_component(THIS_APP_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "Example application, ${THIS_APP_NAME}, is ready to build")

link_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_executable(${THIS_APP_NAME} ${THIS_APP_SOURCES} ${THIS_APP_HEADERS})
target_link_libraries(${THIS_APP_NAME} pthread rt esmacat_core esmacat_driver)
set_target_properties(${THIS_APP_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/tmp/run.sh
"sudo ./${THIS_APP_NAME} &
sudo ${PROJECT_BINARY_DIR}/esmacat_core/application/ecat_main")

file (COPY ${CMAKE_CURRENT_BINARY_DIR}/tmp/run.sh DESTINATION ${CMAKE_CURRENT_BINARY_DIR} FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ)
file(REMOVE_RECURSE ${CMAKE_CURRENT_BINARY_DIR}/tmp)

add_executable(${THIS_APP_NAME}_interface sensojoint_shared_memory.cpp sensojoint_shared_memory.h interface_main.cpp sensojoint_interface.cpp sensojoint_interface.h sensojoint_structs.h )
add_executable(${THIS_APP_NAME}_display PROVAH.h sensojoint_shared_memory.cpp sensojoint_shared_memory.h display_main.cpp sensojoint_structs.h PROVAH2.h)

# FES Control
file(GLOB REHAMOVE_SOURCES ${CMAKE_SOURCE_DIR}/esmacat_applications/sensojoint_master_FES/stimulator_libraries/*.cpp)
file(GLOB REHAMOVE_HEADERS ${CMAKE_SOURCE_DIR}/esmacat_applications/sensojoint_master_FES/stimulator_libraries/*.h)
add_executable(${THIS_APP_NAME}_FES sensojoint_shared_memory.cpp sensojoint_shared_memory.h main_fes.cpp headers_fes.h stimulation.cpp stimulation.h sensojoint_fes_manager.cpp sensojoint_fes_manager.h sensojoint_fes_app.h sensojoint_fes_app.cpp loop_time_stats.cpp   ${REHAMOVE_SOURCES} ${REHAMOVE_HEADERS} ${THIS_APP_HEADERS} )
#add_executable(${THIS_APP_NAME}_FES sensojoint_shared_memory.cpp sensojoint_shared_memory.h main_fes.cpp headers.h stimulation.cpp stimulation.h ${REHAMOVE_SOURCES} ${REHAMOVE_HEADERS} )

# Add FES Control Libraries
include_directories(resource)
include_directories(stimulator_libraries)
add_library(libsmpt STATIC IMPORTED)
set_target_properties(libsmpt PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/stimulator_libraries/libsmpt.a)
target_link_libraries(${THIS_APP_NAME}_FES pthread rt libsmpt)
