## =========================================================================
## @author Leonardo Florez-Valencia (florez-l@javeriana.edu.co)
## =========================================================================

file(GLOB_RECURSE _demo_hdr "${CMAKE_CURRENT_SOURCE_DIR}/*.h*")
file(GLOB_RECURSE _demo_src "${CMAKE_CURRENT_SOURCE_DIR}/*.c*")

include_directories(
  ${BULLET_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${OIS_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/lib/ogrebullet/Collisions/include
  ${PROJECT_SOURCE_DIR}/lib/ogrebullet/Dynamics/include
  ${PROJECT_SOURCE_DIR}/demos/ogrebullet/common/include
  )
add_executable(miAplicacioncita ${_demo_src})

## eof - $RCSfile$
