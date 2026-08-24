##
## BRIDGE/CMakeLists.txt
##
## this directory exports BRIDGE_library
##
## Note includes 4 files from vendor/f710_control_v1
##

set(BRIDGE_LIBRARY_SOURCE_FILES
	asio/outputter.cpp
	asio/serial_bridge.cpp
	threaded/outputter.cpp
	serial_settings.cpp
	${RBL_DIR}/cv_queue.cpp
	${RBL_DIR}/iobuffer.cpp
	${RBL_DIR}/logger.cpp
	${RBL_DIR}/std_format.cpp
)
set(BRIDGE_LIBRARY_HEADER_FILES
	asio/outputter.h 
	asio/serial_bridge.h
	threaded/outputter.h 
	serial_settings.h 
	${RBL_DIR}/cv_queue.h
	${RBL_DIR}/iobuffer.h
	${RBL_DIR}/logger.h
	${RBL_DIR}/std_format.h
)

set_source_files_properties(${BRIDGE_LIBRARY_SOURCE_FILES} PROPERTIES LANGUAGE CXX)

add_library(BRIDGE_library 
	STATIC 
		${BRIDGE_LIBRARY_SOURCE_FILES} 
)

target_include_directories(BRIDGE_library 
	SYSTEM PUBLIC 
	${BRIDGE_INCLUDE_PATHS}
)

