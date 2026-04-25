cmake_minimum_required(VERSION 3.8)
project(vla)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install executable Python node scripts
install(PROGRAMS
  vla/pick_and_place_moveit.py
  vla/demo_recorder.py
  vla/smolvla_inference_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()