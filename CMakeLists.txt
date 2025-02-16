cmake_minimum_required(VERSION 3.8)
project(phntm_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
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
# find_package(OpenSSL)
find_library(OPENSSL_LIBRARY ssl)
find_library(OPENSSL_CRYPTO_LIBRARY crypto)

add_definitions(
    -DSIO_TLS
    # -DDEBUG

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
              src/lib.cpp
              src/git.cpp
              src/sio.cpp
              src/introspection.cpp
              src/services.cpp
              src/phntm_bridge.cpp
              src/config.cpp
              src/extra_packages.cpp
              ${SIOCLIENT_SRC}
              # ${ASIO_SRC}
              )
# target_compile_definitions(phntm_bridge PUBLIC -DSIO_TLS)

target_include_directories(phntm_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /root/websocketpp/
  # /usr/include/asio/
  /root/asio/asio
  ${TINYXML2_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
)
target_compile_features(phntm_bridge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(phntm_bridge
                          rclcpp
                          std_msgs
                          sensor_msgs
                          std_srvs
                          ffmpeg_image_transport_msgs
                          phntm_interfaces
                          )

target_link_libraries(phntm_bridge
  ${FMT_LIBRARY}
  ${JSONCPP_LIBRARY}
  ${OPENSSL_LIBRARY}
  ${OPENSSL_CRYPTO_LIBRARY}
  ${YAML_CPP_LIBRARIES}
  ${TINYXML2_LIBRARY}
  # ${WEBSOCKET_LIBRARY}
  # ${ASIO_LIBRARY}
  # ${RAPIDJSON_LIBRARY}
  # ${SIOCLIENT_LIBRARY}
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
