cmake_minimum_required(VERSION 3.16)
project(lio_web LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
ament_python_install_package(${PROJECT_NAME})
pybind11_add_module(_native src/native.cpp)
target_compile_features(_native PRIVATE cxx_std_17)
target_compile_options(_native PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
install(TARGETS _native LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
install(PROGRAMS scripts/lio_web_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY web launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # ament's build-tree Python symlink contains no installed native library.
  # Run outside that directory so tests exercise the complete installed package.
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/pytest_work")
  ament_add_pytest_test(lio_web_tests test TIMEOUT 60
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/pytest_work")
endif()
ament_package()
