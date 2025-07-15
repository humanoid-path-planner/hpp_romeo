#
# Copyright (c) 2014, 2020 CNRS Authors: Renaud Viry, Guilhem Saurel
#

cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME hpp_romeo)
set(PROJECT_DESCRIPTION "Python and ros launch files for Romeo robot in hpp.")

if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif(NOT BUILD_STANDALONE_PYTHON_INTERFACE)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
  message(STATUS "JRL cmakemodules found in 'cmake/' git submodule")
else()
  find_package(jrl-cmakemodules QUIET CONFIG)
  if(jrl-cmakemodules_FOUND)
    get_property(
      JRL_CMAKE_MODULES
      TARGET jrl-cmakemodules::jrl-cmakemodules
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

include("${JRL_CMAKE_MODULES}/hpp.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

if(BUILD_PYTHON_INTERFACE)
  findpython(REQUIRED)
endif(BUILD_PYTHON_INTERFACE)

add_project_dependency("hpp-corbaserver" REQUIRED)
add_project_dependency("example-robot-data" REQUIRED)

if(BUILD_PYTHON_INTERFACE)
  install(FILES src/hpp/corbaserver/romeo/robot.py
                src/hpp/corbaserver/romeo/__init__.py
          DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/romeo)

  install(FILES src/hpp/corbaserver/manipulation/romeo/robot.py
                src/hpp/corbaserver/manipulation/romeo/__init__.py
          DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation/romeo)
endif()

if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  add_library(${PROJECT_NAME} INTERFACE)
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif()
