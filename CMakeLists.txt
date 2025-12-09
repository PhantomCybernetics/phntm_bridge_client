cmake_minimum_required(VERSION 3.8)
project(phntm_bridge)

# generates compile_commands.json for use with clangd (inside the build directory)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# debug symbols
add_compile_options(-g)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(phntm_interfaces REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_library(FMT_LIBRARY fmt)
find_library(JSONCPP_LIBRARY jsoncpp)
find_package(yaml-cpp REQUIRED)
find_library(TINYXML2_LIBRARY tinyxml2)
find_library(UUID_LIBRARY uuid)
# find_package(OpenSSL)
find_library(OPENSSL_LIBRARY ssl)
find_library(OPENSSL_CRYPTO_LIBRARY crypto)
find_package(CURL REQUIRED)
find_package(LibDataChannel REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiodcxx)

pkg_check_modules(FFMPEG REQUIRED 
    libavformat 
    libavcodec 
    libavutil 
    libswscale
)

find_library(AVFORMAT NAMES avformat PATHS /lib/aarch64-linux-gnu)

add_definitions(
    -DSIO_TLS
    # -DDEBUG
    # -DCMAKE_BUILD_TYPE=Debug

    # These will force ASIO to compile without Boost
    # -DBOOST_DATE_TIME_NO_LIB
    # -DBOOST_REGEX_NO_LIB
    -DASIO_STANDALONE
    # These will force sioclient to compile with C++11
    -D_WEBSOCKETPP_CPP11_STL_
    -D_WEBSOCKETPP_CPP11_FUNCTIONAL_
    -D_WEBSOCKETPP_CPP11_TYPE_TRAITS_
    -D_WEBSOCKETPP_CPP11_CHRONO_
)

# find_library(SIOCLIENT_LIBRARY sioclient)

# find_library(WEBSOCKET_LIBRARY websocketcpp)
# find_library(ASIO_LIBRARY asiolib)
# find_library(RAPIDJSON_LIBRARY RapidJSON)

include_directories(/usr/include/jsoncpp/)
include_directories(${GPIOD_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${FFMPEG_INCLUDE_DIRS})

set(SIOCLIENT_SRC
  "sioclient/sio_client.cpp"
  "sioclient/sio_socket.cpp"
  "sioclient/internal/sio_client_impl.cpp"
  "sioclient/internal/sio_packet.cpp"
)

# set(ASIO_SRC
#   "/root/asio-1.30.2/src/asio_ssl.cpp"
# )

add_executable(phntm_bridge
              src/const.cpp
              src/lib.cpp
              src/git.cpp
              src/sio.cpp
              src/introspection.cpp
              src/generic_services.cpp
              src/local_services.cpp
              src/phntm_bridge.cpp
              src/topic_reader_data.cpp
              src/topic_reader_h264.cpp
              src/topic_writer_data.cpp
              src/config.cpp
              src/extra_packages.cpp
              src/wrtc_peer.cpp
              src/status_leds.cpp
              src/file_extractor.cpp
              src/ffmpeg_encoder.cpp
              ${SIOCLIENT_SRC}
              # ${ASIO_SRC}
              )
# target_compile_definitions(phntm_bridge PUBLIC -DSIO_TLS)


target_include_directories(phntm_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /root/websocketpp/
  /usr/include/
  /ros2_ws/src/phntm_bridge/sioclient/
  /root/asio/asio/include/
  /root/libdatachannel/include/
  # /root/libdatachannel/include/rtc/
  # /root/libdatachannel/deps/plog/include/
  # /root/libdatachannel/src/
  ${TINYXML2_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
  ${GPIOD_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIR}
  ${FFMPEG_INCLUDE_DIRS}
)
target_compile_features(phntm_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#ament_target_dependencies(phntm_bridge
#                          ament_index_cpp
#                          rclcpp
#                          std_msgs
#                          sensor_msgs
#                          std_srvs
#                          ffmpeg_image_transport_msgs
#                          phntm_interfaces
#                          )

link_directories(${FFMPEG_LIBRARY_DIRS})

set(FFMPEG_LINK_FLAGS
    -lavformat
    -lavcodec
    -lavutil
    -lswscale
    -lswresample
    -lavdevice
    -lavfilter
)

target_link_libraries(phntm_bridge
  ${ffmpeg_image_transport_msgs_TARGETS}
  ${phntm_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  ${FMT_LIBRARY}
  ${JSONCPP_LIBRARY}
  ${OPENSSL_LIBRARY}
  ${OPENSSL_CRYPTO_LIBRARY}
  ${YAML_CPP_LIBRARIES}
  ${TINYXML2_LIBRARY}
  ${GPIOD_LIBRARIES}
  ${UUID_LIBRARY}
  ${CURL_LIBRARIES}
  LibDataChannel::LibDataChannel
  ${OpenCV_LIBS}
  ${FFMPEG_LINK_FLAGS}
  ${AVFORMAT}
  # ${WEBSOCKET_LIBRARY}
  # ${ASIO_LIBRARY}
  # ${RAPIDJSON_LIBRARY}
  # ${SIOCLIENT_LIBRARY}
  -lpthread
  -ldl
  -lz
  -lm
)

link_directories(${GPIOD_LIBRARY_DIRS})

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
