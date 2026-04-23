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
  vla/vla_action_client.py
  vla/fake_vla_action_client.py
  vla/action_to_ee.py
  vla/vla_ik.py
  vla/grasp_spike_local.py
  vla/pick_and_place_moveit.py
  vla/demo_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()