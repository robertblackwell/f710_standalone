target_sources(applib PRIVATE
	fd_functions.cpp
	serial_settings.cpp
)

add_subdirectory(asio)
add_subdirectory(threaded)
