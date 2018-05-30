## =========================================================================
## @author Leonardo Florez-Valencia (florez-l@javeriana.edu.co)
## =========================================================================

include_directories(
  ${BULLET_INCLUDE_DIR}
  ${OIS_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/lib
  ${PROJECT_BINARY_DIR}/lib
  ${PROJECT_SOURCE_DIR}/lib/pujOgreBullet/Dynamics/include
  ${PROJECT_SOURCE_DIR}/lib/pujOgreBullet/Collisions/include
  ${PROJECT_SOURCE_DIR}/lib/pujOgreBullet/Collisions/include/Shapes
  ${PROJECT_SOURCE_DIR}/lib/pujOgreBullet/Collisions/include/Utils
  )
add_executable(RagDollApp ragdoll.cxx)
target_link_libraries(RagDollApp pujOgreBullet)

## eof - $RCSfile$
