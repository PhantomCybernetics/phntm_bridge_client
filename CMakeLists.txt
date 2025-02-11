cmake_minimum_required(VERSION 3.8)
project(phntm_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_library(FMT_LIBRARY fmt)
find_library(JSONCPP_LIBRARY jsoncpp)

include_directories(/usr/include/jsoncpp/)

add_executable(phntm_bridge
              src/lib.cpp
              src/git.cpp
              src/services.cpp
              src/phntm_bridge.cpp
              src/config.cpp)

target_include_directories(phntm_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(phntm_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(phntm_bridge
                          rclcpp
                          std_msgs
                          sensor_msgs
                          std_srvs
                          ffmpeg_image_transport_msgs
                          )

target_link_libraries(phntm_bridge
  ${FMT_LIBRARY}
  ${JSONCPP_LIBRARY}
)

install(TARGETS phntm_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
