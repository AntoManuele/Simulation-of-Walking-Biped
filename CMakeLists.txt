cmake_minimum_required(VERSION 2.8.3)

project(biped_sensor)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		)
catkin_package()

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(walk biped_control/src/zwInit.cpp)
target_link_libraries(walk ${catkin_LIBRARIES})
add_dependencies(walk ${catkin_EXPORTED_TARGETS})