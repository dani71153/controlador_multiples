cmake_minimum_required(VERSION 3.8)
project(controlador_multiples)

# Política opcional (similar al ejemplo grande)
cmake_policy(SET CMP0009 NEW)

# Opciones de compilación
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------
# Encontrar dependencias ROS2
# ---------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# NUEVO: Dependencias para TF2
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---------------------------
# Ejecutable del nodo
# ---------------------------
add_executable(controlador_multiple
  src/controlador_multiple.cpp
)

target_include_directories(controlador_multiple PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(controlador_multiple PUBLIC c_std_99 cxx_std_17)

# ---------------------------
# Dependencias del ejecutable
# ---------------------------
ament_target_dependencies(controlador_multiple
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  std_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

# ---------------------------
# Instalación del ejecutable
# ---------------------------
install(TARGETS controlador_multiple
  DESTINATION lib/${PROJECT_NAME}
)

# (Opcional) instalación de carpeta launch/
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------
# Testing / Linting
# ---------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ---------------------------
# Final del paquete
# ---------------------------
ament_package()
