cmake_minimum_required(VERSION 3.8)
project(rec_rep2_servo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetStatus.srv"
)

# ── Portable core: friction observer, safety monitor, gravity model. ───────
# No dependency on the Kortex SDK -- always builds, always testable.
add_library(rec_rep2_servo_core
  src/friction_observer.cpp
  src/safety_monitor.cpp
  src/gravity_model.cpp
)
target_include_directories(rec_rep2_servo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Pinocchio exports modern CMake imported targets rather than the classic
# ament_cmake INCLUDE_DIRS/LIBRARIES style, so link its target directly
# instead of going through ament_target_dependencies().
target_link_libraries(rec_rep2_servo_core PUBLIC pinocchio::pinocchio)

install(
  TARGETS rec_rep2_servo_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pinocchio)

# ── compliant_torque_node: requires the vendor Kortex C++ SDK. ─────────────
# Kept optional so `colcon build` never hard-fails on a dev machine that
# doesn't have the SDK installed -- see docs/cpp_servoing.md.
option(BUILD_KORTEX_SERVO_NODE
  "Build compliant_torque_node (requires the vendor Kortex C++ SDK)" ON)

if(BUILD_KORTEX_SERVO_NODE)
  # TODO: verify against the vendor SDK -- header/library names and the
  # KORTEX_API_DIR layout below are inferred from the public Kortex C++
  # examples, not confirmed against an installed copy of the SDK.
  find_path(KORTEX_API_INCLUDE_DIR
    NAMES BaseClientRpc.h
    PATHS "$ENV{KORTEX_API_DIR}/include" "${KORTEX_API_DIR}/include"
    PATH_SUFFIXES client common messages
  )
  find_library(KORTEX_API_LIBRARY
    NAMES KortexApiCpp KortexApi
    PATHS "$ENV{KORTEX_API_DIR}/lib/release" "${KORTEX_API_DIR}/lib/release"
  )

  if(NOT KORTEX_API_INCLUDE_DIR OR NOT KORTEX_API_LIBRARY)
    message(WARNING
      "rec_rep2_servo: Kortex C++ SDK not found (set KORTEX_API_DIR to its "
      "unpacked location) -- skipping compliant_torque_node executable. "
      "rec_rep2_servo_core and its tests still build. "
      "See docs/cpp_servoing.md.")
  else()
    add_executable(compliant_torque_node
      src/kortex_interface.cpp
      src/compliant_torque_node.cpp
      src/main.cpp
    )
    target_include_directories(compliant_torque_node PRIVATE ${KORTEX_API_INCLUDE_DIR})
    target_link_libraries(compliant_torque_node
      rec_rep2_servo_core
      ${KORTEX_API_LIBRARY}
    )
    ament_target_dependencies(compliant_torque_node PUBLIC
      rclcpp std_srvs ament_index_cpp
    )
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(compliant_torque_node "${cpp_typesupport_target}")

    install(TARGETS compliant_torque_node DESTINATION lib/${PROJECT_NAME})
  endif()
endif()

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_friction_observer test/test_friction_observer.cpp)
  target_link_libraries(test_friction_observer rec_rep2_servo_core)

  ament_add_gtest(test_safety_monitor test/test_safety_monitor.cpp)
  target_link_libraries(test_safety_monitor rec_rep2_servo_core)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
