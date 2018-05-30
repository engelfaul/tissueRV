## =========================================================================
## @author Leonardo Florez-Valencia (florez-l@javeriana.edu.co)
## =========================================================================

include_directories(
  ${OIS_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/lib
  ${PROJECT_BINARY_DIR}/lib
  )
add_executable(NinjaApp ninja.cxx)
target_link_libraries(NinjaApp pujOgre)

## eof - $RCSfile$
