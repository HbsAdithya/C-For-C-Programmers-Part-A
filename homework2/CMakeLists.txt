#
# Author: Michael Melachridis
#

# ---------------------------- Defining Project Properties ----------------------------
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT (muce)
SET (APPLICATION_NAME "Dikstra's Shorting Algorithm DEMO")
SET (APPLICATION_CODENAME "${PROJECT_NAME}")
SET (APPLICATION_COPYRIGHT_YEARS "2017")
SET (APPLICATION_VERSION_MAJOR 1)
SET (APPLICATION_VERSION_MINOR 0)
SET (APPLICATION_VERSION_PATCH 0)
SET (APPLICATION_VERSION_TYPE SNAPSHOT)
SET (APPLICATION_VERSION_STRING "${APPLICATION_VERSION_MAJOR}.${APPLICATION_VERSION_MINOR}.${APPLICATION_VERSION_PATCH}-${APPLICATION_VERSION_TYPE}")
SET (APPLICATION_VENDOR_ID "michael.melachridis")
SET (APPLICATION_VENDOR_NAME "Michael Melachridis")
#SET (APPLICATION_VENDOR_URL "yourcompany.com")
SET (APPLICATION_ID "${APPLICATION_VENDOR_ID}.${PROJECT_NAME}")

# ---------------------------- Defining building options ------------------------------
include (${CMAKE_ROOT}/Modules/CheckFunctionExists.cmake)
# Instruct CMake to run moc automatically when needed.
SET (CMAKE_AUTOMOC ON)

# ---------------------------- Defining Compiler options ------------------------------
SET (CMAKE_INCLUDE_CURRENT_DIR ON)
SET (CMAKE_CXX_COMPILER "/usr/bin/g++")
SET (CMAKE_C_COMPILER "/usr/bin/gcc")

# ---------------------------- Defining Debugging Options ------------------------------
SET (CMAKE_VERBOSE_MAKEFILE 1) # Use 1 for debugging, 0 for release
MESSAGE (STATUS "compiler is ${CMAKE_CXX_COMPILER_ID}")

# ---------------------------- Defining Project Output Paths ---------------------------
SET (MAINFOLDER ${PROJECT_SOURCE_DIR})
SET (EXECUTABLE_OUTPUT_PATH "${MAINFOLDER}/../bin")
SET (LIBRARY_OUTPUT_PATH "${MAINFOLDER}/../lib")


# ---------------------------- Defining Project Search Paths ---------------------------
LIST (APPEND CMAKE_PREFIX_PATH "${MAINFOLDER}")
LIST (APPEND CMAKE_PREFIX_PATH "${MAINFOLDER}/../tools")
LIST (APPEND CMAKE_PREFIX_PATH "${MAINFOLDER}/../3pp")

# ---------------------------- Defining Compiler Flags ---------------------------------
SET(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
SET(GCC_COVERAGE_LINK_FLAGS    "-lgcov")
SET (CPPFLAGS_COMMON "-Wall -Wextra -Wundef -Wendif-labels")
SET (CXXFLAGS_COMMON "-std=c++11 -m64 -Wl,--allow-shlib-undefined -g -O2 -pthread -fPIC -fmessage-length=0")
SET (CFLAGS_COMMON "${CXXFLAGS}")
SET (CPPFLAGS "${CPPFLAGS_COMMON}")
SET (CXXFLAGS "${CXXFLAGS_COMMON}")
SET (CFLAGS "${CFLAGS_COMMON}")
SET (CMAKE_CXX_FLAGS "${CPPFLAGS} ${CXXFLAGS}")
#ADD_DEFINITIONS (${GCC_COVERAGE_COMPILE_FLAGS})


# ------------------------- Defining Application Definitions ---------------------------
#SET (PROJECT_DEFINES "-DBOOST_LOG_DYN_LINK")
#SET (CXXUNITTESTFLAGS "-fprofile-arcs -ftest-coverage")
ADD_DEFINITIONS (${PROJECT_DEFINES} ${CXXUNITTESTFLAGS})

# ---------------------------- Checking the OS -----------------------------------------
IF("${CMAKE_SYSTEM}" MATCHES "Linux")
	MESSAGE(STATUS "Compiling on Linux")
	SET(LINUX 1)
	EXEC_PROGRAM("uname" ARGS -m OUTPUT_VARIABLE myArch)
	IF("${myArch}" MATCHES "x86_64")
		MESSAGE(STATUS "  64-bit architecture detected")
		SET(LINUX64 1)
 		SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}64")
 		SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_OUTPUT_DIRECTORY}64")
	ENDIF("${myArch}" MATCHES "x86_64")
ELSEIF(APPLE)
	SET(LINUX 0)
	MESSAGE(STATUS "Compiling on MacOSX")
ELSEIF(WIN32)
	SET(LINUX 0)
	IF(CYGWIN)
		MESSAGE(STATUS "Compiling on CygWin")
		SET(CYGWIN 1)
	ELSE(CYGWIN)
		MESSAGE(FATAL_ERROR "Compiling on Windows is not possible at the moment")
	ENDIF(CYGWIN)
ENDIF("${CMAKE_SYSTEM}" MATCHES "Linux")

# ---------------------------- Defining Environmental Variables -------------------------------------
#SET (LDPATH "")
#SET (ENV{LD_LIBRARY_PATH} ${LDPATH})

# ---------------------------- Defining Build Targets -------------------------------------
ADD_EXECUTABLE (Dijkstra Dijkstra.cpp Graph.cpp ShortestPathAlgo.cpp)

# ---------------------------- Defining custom targets -----------------------------
SET (BINDIR ${EXECUTABLE_OUTPUT_PATH})
SET (BINARY ${BINDIR}/Dikjstra)

