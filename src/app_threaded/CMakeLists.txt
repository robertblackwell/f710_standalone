
add_executable(f710_controller_v1 
	main.cpp
	outputter.cpp
	serial_link.cpp
	f710.cpp

	)
target_link_libraries(f710_controller_v1 PUBLIC applib)

add_executable(f710_threaded_demo
	demo.cpp
	outputter.cpp
	f710.cpp
)
target_link_libraries(f710_threaded_demo PUBLIC applib)
#target_include_directories(${TARGET} PUBLIC ${APP_INCLUDE_DIRS})
