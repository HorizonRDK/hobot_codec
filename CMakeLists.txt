cmake_minimum_required(VERSION 3.5)
project(hobot_codec)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

OPTION(SHARED_MEM    "SHARED_MEM work environment"    OFF)

if(SHARED_MEM)
add_definitions(-DUSING_HBMEM)
find_package(hbm_img_msgs REQUIRED)
add_definitions(-DSHARED_MEM_MSG)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(img_msgs REQUIRED)

include_directories(include
${PROJECT_SOURCE_DIR}
${CMAKE_SYSROOT}/usr/include
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbmedia
)

add_executable(hobot_codec_republish
src/example.cpp
src/hobot_codec.cpp
src/hobot_vdec.cpp
src/hobot_venc.cpp
)

set(BASE_LIBRARIES pthread vio dl)
set(X3_HAPI_LIB isp_algo z hbmedia isp ion multimedia iar tinyalsa cjson)

#${PROJECT_NAME}
target_link_libraries(hobot_codec_republish
  ${BASE_LIBRARIES} ${X3_HAPI_LIB}
)

if(SHARED_MEM)
target_link_libraries(hobot_codec_republish
  hbmem alog
)
ament_target_dependencies(
  hobot_codec_republish
  rclcpp
  sensor_msgs
  img_msgs
  hbm_img_msgs
)
else()
ament_target_dependencies(
  hobot_codec_republish
  rclcpp
  img_msgs
  sensor_msgs
)
endif()

target_link_libraries(hobot_codec_republish
)

# Install executables
install(
  TARGETS hobot_codec_republish
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
