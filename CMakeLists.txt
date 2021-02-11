
project(integrity_toolkit CXX C)
cmake_minimum_required(VERSION 3.5)

set (INTEGRITY_TOOLKIT_MAJOR_VERSION 3)
set (INTEGRITY_TOOLKIT_MINOR_VERSION 0)
set (INTEGRITY_TOOLKIT_PATCH_VERSION 0)

set (PROJECT_VERSION ${INTEGRITY_TOOLKIT_MAJOR_VERSION}.${INTEGRITY_TOOLKIT_MINOR_VERSION}.${INTEGRITY_TOOLKIT_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${PROJECT_VERSION}")

option(BUILD_KIT "Build kit?" false)

set(as_subproject logutils if_utils pnt_integrity )
if(BUILD_KIT)
  set(as_subproject logutils if_utils pnt_integrity integrity_ui_base serial 
  ublox yaml_parser usrp_utilities)
endif(BUILD_KIT)

# Allow dependencies to be found when added using add_subdirectory
# https://gitlab.kitware.com/cmake/cmake/issues/17735
macro(find_package)
    if(NOT "${ARGV0}" IN_LIST as_subproject)
        _find_package(${ARGV})
    endif()
endmacro()

# build packages
add_subdirectory(is4s_common/logutils)
add_subdirectory(if_data_utils)
add_subdirectory(pnt_integrity/pnt_integrity)

if(BUILD_KIT)
  add_subdirectory(integrity_ui/integrity_ui_base)
  add_subdirectory(is4s_common/yaml_parser)
  add_subdirectory(serial)
  add_subdirectory(ublox/ublox)
  add_subdirectory(usrp_utilities/usrp_utilities)

  ###############################################################################
  ## Configure compiler                                                        ##
  ###############################################################################

  set(CMAKE_INCLUDE_CURRENT_DIR ON)

  set(CMAKE_AUTOUIC ON)
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTORCC ON)

  set(CMAKE_CXX_FLAGS "-fPIC")

  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  # ###############################################################################
  # ## Find dependencies                                                         ##
  # ###############################################################################
  find_package(integrity_ui_base REQUIRED)
  find_package(pnt_integrity REQUIRED)
  find_package(Qt5 COMPONENTS Widgets WebEngineWidgets Charts REQUIRED)
  find_package(logutils REQUIRED)
  find_package(serial REQUIRED)
  find_package(ublox REQUIRED)
  find_package(yaml_parser REQUIRED)
  find_package(usrp_utilities REQUIRED)

  ###############################################################################
  ## Build Application                                                         ##
  ###############################################################################

  include_directories(include ${QT_USE_FILE} )

  if (WIN32)
    set (EXECUTABLE_ARGS WIN32)
  else()
    set (EXECUTABLE_ARGS)
  endif()

  add_executable(${PROJECT_NAME} ${EXECUTABLE_ARGS}
    include/integrity_toolkit/ToolkitApplication.hpp
    src/ToolkitApplication.cpp
    src/toolkitMainApplication.cpp
  )

  target_link_libraries(${PROJECT_NAME} PRIVATE integrity_ui_base pnt_integrity ublox 
    usrp_utilities yaml_parser Qt5::Widgets Qt5::Charts Qt5::WebEngineWidgets)

  if(PNT_INTEGRITY_INCLUDES_ACQ_CHECK)
    target_compile_definitions(${PROJECT_NAME} PRIVATE PNT_INTEGRITY_INCLUDES_ACQ_CHECK)
  endif()

  install(TARGETS ${PROJECT_NAME} DESTINATION bin)
  install(FILES config/toolkitAppConfig.yaml DESTINATION bin)
endif(BUILD_KIT)
