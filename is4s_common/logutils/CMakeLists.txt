cmake_minimum_required(VERSION 3.6)
project(logutils)


set (LOGUTILS_MAJOR_VERSION 1)
set (LOGUTILS_MINOR_VERSION 10)
set (LOGUTILS_PATCH_VERSION 0)

set (PROJECT_VERSION ${LOGUTILS_MAJOR_VERSION}.${LOGUTILS_MINOR_VERSION}.${LOGUTILS_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${PROJECT_VERSION}")

###############################################################################
## Set compiler options                                                      ##
###############################################################################
# suppress policy warnings on newer versions of cmake
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0053 NEW)
cmake_policy(SET CMP0057 NEW)

###############################################################################
## Set source files                                                          ##
###############################################################################

set(LOGUTILS_SRCS
    src/logutils.cpp
)

set(LOGUTILS_HDRS
    include/logutils/logutils.hpp
)

include_directories(include/)


## Declare a cpp library
add_library(${PROJECT_NAME} ${LOGUTILS_SRCS} ${LOGUTILS_HDRS})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
# Set default compile flags for GCC
if(NOT MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wno-psabi>)

set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<INSTALL_INTERFACE:include>    
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

###############################################################################
## Install targets                                                           ##
###############################################################################

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
        EXPORT logutilsTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    logutilsConfigVersion.cmake
    COMPATIBILITY AnyNewerVersion
)

export(TARGETS ${PROJECT_NAME} FILE logutilsTargets.cmake)

install(EXPORT logutilsTargets
  FILE logutilsTargets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

set(PKG_NAME ${PROJECT_NAME})
configure_package_config_file("cmake/logutilsConfig.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/logutilsConfig.cmake"
                              INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
                              PATH_VARS PKG_NAME
                              NO_SET_AND_CHECK_MACRO
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/logutilsConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/logutilsConfigVersion.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME}
        )