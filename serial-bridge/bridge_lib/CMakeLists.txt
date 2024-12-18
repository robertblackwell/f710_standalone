##
## BRIDGE/CMakeLists.txt
##
## this directory exports BRIDGE_library
##
## Note includes 4 files from vendor/f710_control_v1
##
set(EXTSRC ${BRIDGE_VENDOR_SRCDIR})

set(BRIDGE_LIBRARY_HEADER_FILES
		iobuffer.h
	)

set(BRIDGE_LIBRARY_SOURCE_FILES
		iobuffer.cpp
	)

set_source_files_properties(${BRIDGE_LIBRARY_SOURCE_FILES} PROPERTIES LANGUAGE CXX)

add_library(BRIDGE_library 
	STATIC 
		${BRIDGE_LIBRARY_SOURCE_FILES} 
		${BRIDGE_LIBRARY_HEADER_FILES}
)

target_include_directories(BRIDGE_library 
	SYSTEM PUBLIC 
	${BRIDGE_INCLUDE_PATHS}
)

