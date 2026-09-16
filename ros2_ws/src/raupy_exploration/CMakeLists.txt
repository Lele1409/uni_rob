cmake_minimum_required(VERSION 3.8)
project(raupy_exploration)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python helpers (pure logic, importable by the supervisor and by tests)
ament_python_install_package(${PROJECT_NAME})

# Executables
install(PROGRAMS
  scripts/exploration_supervisor.py
  scripts/fake_raupy.py
  scripts/scan_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Configs, launch files, RViz layouts, map output folder
install(DIRECTORY config launch rviz maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  file(GLOB _pytests test/test_*.py)
  foreach(_t ${_pytests})
    get_filename_component(_name ${_t} NAME_WE)
    ament_add_pytest_test(${_name} ${_t})
  endforeach()
endif()

ament_package()
