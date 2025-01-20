cmake_minimum_required(VERSION 3.5)
project(ar_controller)

find_package(ament_cmake REQUIRED)

install(

  DIRECTORY config scripts 

  DESTINATION share/${PROJECT_NAME}

)

install(PROGRAMS  
  scripts/__init__.py
  scripts/control.py
  scripts/keyboard.py
  scripts/inverse_kinematic.py
  scripts/hand.py
  DESTINATION lib/${PROJECT_NAME} )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
