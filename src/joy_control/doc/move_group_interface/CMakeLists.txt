add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#add new part
add_executable(joy_to_arm src/joy_to_arm.cpp)
target_link_libraries(joy_to_arm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS joy_to_arm DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#end
