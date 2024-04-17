# Esmacat Node
add_executable(esmacat_node_2_ease esmacat_main.cpp my_app.cpp my_app.h ${esmacat_ros_SOURCES} ${esmacat_ros_HEADERS})
add_dependencies(esmacat_node_2_ease ${catkin_EXPORTED_TARGETS} esmacat_high_performance_generate_messages_cpp)

target_link_libraries(esmacat_node_2_ease
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  pthread
  rt
  esmacat_core
  esmacat_driver
)

install(TARGETS esmacat_node_2_ease
        RUNTIME DESTINATION ${PROJECT_BINARY_DIR}/lib
)

# Add Launch file configuration depending on the Hardware
if(NOT (${esmacat_master_hw_type_c} STREQUAL "-1"))
 file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/run.sh
 "source ${PROJECT_BINARY_DIR}/devel/setup.bash \n"
 "roslaunch ${PROJECT_NAME} 2_ease_blink_master_c.launch")
 
elseif(NOT (${esmacat_master_hw_type_s} STREQUAL "-1"))
 file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/run.sh
 "source ${PROJECT_BINARY_DIR}/devel/setup.bash \n"
 "roslaunch ${PROJECT_NAME} 2_ease_blink_master_s.launch")
endif()

