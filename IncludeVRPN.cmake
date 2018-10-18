FIND_PATH(
  VRPN_INCLUDE_DIR
  vrpn_Types.h
  /usr/include
  /usr/local/include
  )
IF(NOT EXISTS ${VRPN_INCLUDE_DIR}/vrpn_Types.h)
  SET(
    VRPN_INCLUDE_DIR
    VRPN_INCLUDE_DIR-NOTFOUND
    CACHE PATH "path to vrpn_*.h"
    FORCE
    )
  MESSAGE(FATAL_ERROR "VRPN headers not found!")
ENDIF(NOT EXISTS ${VRPN_INCLUDE_DIR}/vrpn_Types.h)
INCLUDE_DIRECTORIES(${VRPN_INCLUDE_DIR})

FIND_LIBRARY(
  VRPN_LIBRARY
  NAMES
  vrpn
  PATHS
  /usr/lib
  /usr/local/lib
  )
IF(NOT VRPN_LIBRARY)
  MESSAGE(FATAL_ERROR "VRPN library not found!")
ENDIF(NOT VRPN_LIBRARY)

## eof - IncludeVRPN.cmake
