
add_executable(f710_asio_controller 
	main.cpp
	outputter.cpp
	serial_bridge.cpp
	model.cpp
	)
target_link_libraries(f710_asio_controller PUBLIC applib)

add_executable(f710_asio_demo
	demo.cpp
	outputter.cpp
	model.cpp
)
target_link_libraries(f710_asio_demo PUBLIC applib)
#target_include_directories(${TARGET} PUBLIC ${APP_INCLUDE_DIRS})
