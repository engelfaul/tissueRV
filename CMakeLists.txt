## =========================================================================
## @author Leonardo Florez-Valencia (florez-l@javeriana.edu.co)
## =========================================================================

## == Base project configuration
cmake_minimum_required(VERSION 3.0)
set(prj_NAME pujOgre)
set(prj_MAJ 1)
set(prj_MIN 0)
set(prj_REL 0)

## == Project definiton
project(${prj_NAME} VERSION "${prj_MAJ}.${prj_MIN}.${prj_REL}")

## == If working on a MacOSX, activate the use of RPATH's
## == Furthermore: prepare the type of executables
set(APP_TYPE "" CACHE STRING "Executable linking." FORCE)
if(APPLE)
  set(APP_TYPE "MACOSX_BUNDLE" CACHE STRING "Executable linking." FORCE)
  set(CMAKE_MACOSX_RPATH true CACHE BOOL "Use RPATH's on MacOSX." FORCE)
  mark_as_advanced(CMAKE_MACOSX_RPATH)
elseif(WIN32)
  set(APP_TYPE "WIN32" CACHE STRING "Executable linking." FORCE)
endif(APPLE)
mark_as_advanced(APP_TYPE)

## == Force c++11
set(CMAKE_CXX_STANDARD 11)

## -------------------------------------------------------------------------
## Find VRPN and OpenHaptics
## -------------------------------------------------------------------------
INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/IncludeVRPN.cmake)
#INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/IncludeOPENHAPTICS.cmake)


## == Prepare header generator to build shared libs
include(GenerateExportHeader)

## == Do not allow to build inside the source tree
if(PROJECT_BINARY_DIR STREQUAL ${PROJECT_SOURCE_DIR})
  message(FATAL_ERROR "Building in the source tree is not allowed.")
endif(PROJECT_BINARY_DIR STREQUAL ${PROJECT_SOURCE_DIR})

## == Where to put targets (executables and libs)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR})
mark_as_advanced(
  CMAKE_BACKWARDS_COMPATIBILITY
  EXECUTABLE_OUTPUT_PATH
  LIBRARY_OUTPUT_PATH
  )

## == Find Ogre
find_package(OGRE REQUIRED)
link_directories(${OGRE_LIBRARY_DIRS})

## == Find Bullet
find_package(Bullet REQUIRED)

## == Find OIS
find_path(
  OIS_INCLUDE_DIRS
  NAMES OISConfig.h
  )
find_library(
  OIS_LIBRARIES
  NAMES OIS
  )

## == Find OpenHaptics
###
# HDAPI: HD
###

find_path(HDAPI_INCLUDE_DIR
	NAMES
	HD/hd.h
	HINTS
${_incsearchdirs})

find_library(HDAPI_LIBRARY
	NAMES
	HD
	PATHS
	/usr/lib
)

###
# HDAPI: HDU
###

find_path(HDAPI_HDU_INCLUDE_DIR
	NAMES
	HDU/hdu.h
	HINTS
	${_incsearchdirs})

find_library(HDAPI_HDU_LIBRARY_RELEASE
	NAMES
	HDU
	PATH_SUFFIXES
	ReleaseAcademicEdition
	Release
	HINTS
${_libsearchdirs})
## == Build
subdirs(lib src)

###
# HLAPI: HL
###
find_path(HLAPI_INCLUDE_DIR
	NAMES
	HL/hl.h
	HINTS
	${_incsearchdirs})

find_library(HLAPI_LIBRARY_RELEASE
	NAMES
	HL
	PATH_SUFFIXES
	ReleaseAcademicEdition
	Release
	HINTS
${_libsearchdirs})

###
# HLAPI: HLU
###
find_path(HLAPI_HLU_INCLUDE_DIR
	NAMES
	HLU/hlu.h
	HINTS
	${_incsearchdirs})

find_library(HLAPI_HLU_LIBRARY_RELEASE
	NAMES
	HLU
	PATH_SUFFIXES
	ReleaseAcademicEdition
	Release
	HINTS
	${_libsearchdirs})


find_path(HLAPI_HLU_SOURCE_DIR
		NAMES
		hlu.cpp
		PATH_SUFFIXES
		src
		src/HLU
		src/HLU/src
		libsrc/HLU
		HINTS
		"${HLAPI_HLU_INCLUDE_DIR}/.."
"${HLAPI_HLU_INCLUDE_DIR}/../share/3DTouch")

## eof - $RCSfile$
