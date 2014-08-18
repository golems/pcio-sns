cmake_minimum_required(VERSION 2.6)
project(pcio)

set(PCIO_MAJOR_VERSION "1")
set(PCIO_MINOR_VERSION "0")
set(PCIO_PATCH_VERSION "0")

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Set the C99 standard for the C files
set(CMAKE_C_FLAGS --std=gnu99)

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either 
# in /usr/lib or /usr/local/lib
link_libraries(ach somatic ntcan ntcanopen amino)

# Add the executables
include_directories(include)
add_executable(pciod pciod.c pcio.c code.c)
add_executable(pcio_util pcio_util.c pcio.c code.c)

# Install it
install(TARGETS pciod pcio_util DESTINATION /usr/local/bin)

###############
# Package Installer
set(CPACK_PACKAGE_NAME "pcio")
set(CPACK_SYSTEM_NAME "i386")
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
    set(CPACK_SYSTEM_NAME "amd64")    
endif()
set(CPACK_PACKAGE_VERSION_MAJOR ${PCIO_MAJOR_VERSION})
set(CPACK_PACKAGE_VERSION_MINOR ${PCIO_MINOR_VERSION})
set(CPACK_PACKAGE_VERSION_PATCH ${PCIO_PATCH_VERSION})
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libach-dev, somatic, libamino-dev")
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_SYSTEM_NAME}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Can Erdogan <cerdogan3@gatech.edu>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "An interface to control the Schunk modules using the somatic library")
set(CPACK_PACKAGE_VENDOR "Georgia Tech Humanoids Lab")
include(CPack)
