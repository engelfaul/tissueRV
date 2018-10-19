FILE(GLOB DEVICES_SRC "${PROJECT_SOURCE_DIR}/src/devices/*.cxx")
GET_FILENAME_COMPONENT(_dir ${VRPN_LIBRARY} DIRECTORY)
ADD_LIBRARY(Devices SHARED ${DEVICES_SRC})
TARGET_LINK_LIBRARIES(Devices ${VRPN_LIBRARY} boost_thread boost_system ${_dir}/libquat.a)

## eof - CMakeLists.txt
