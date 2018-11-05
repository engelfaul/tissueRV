## =========================================================================
## @author Leonardo Florez-Valencia (florez-l@javeriana.edu.co)
## =========================================================================

file(GLOB _ogre_lib_hdr "${CMAKE_CURRENT_SOURCE_DIR}/pujOgre/*.h*")
file(GLOB _ogre_lib_src "${CMAKE_CURRENT_SOURCE_DIR}/pujOgre/*.c*")
file(GLOB_RECURSE _ob_lib_hdr "${CMAKE_CURRENT_SOURCE_DIR}/pujOgreBullet/*.h*")
file(GLOB_RECURSE _ob_lib_src "${CMAKE_CURRENT_SOURCE_DIR}/pujOgreBullet/*.c*")

include_directories(
  ${BULLET_INCLUDE_DIR}
  ${OIS_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/pujOgreBullet/Collisions/include
  ${CMAKE_CURRENT_SOURCE_DIR}/pujOgreBullet/Collisions/include/Shapes
  ${CMAKE_CURRENT_SOURCE_DIR}/pujOgreBullet/Collisions/include/Utils
  ${CMAKE_CURRENT_SOURCE_DIR}/pujOgreBullet/Dynamics/include
)
add_library(pujOgre SHARED ${_ogre_lib_src})
target_link_libraries(pujOgre ${OGRE_LIBRARIES} ${OIS_LIBRARIES})

add_library(pujOgreBullet SHARED ${_ob_lib_src})
target_link_libraries(pujOgreBullet pujOgre ${BULLET_LIBRARIES} ConvexDecomposition ${HDAPI_LIBRARY} PHANToMIO raw1394)

## eof - $RCSfile$
