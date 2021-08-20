cmake_minimum_required(VERSION 3.6)
project(pnt_integrity)

set (PNT_INTEGRITY_MAJOR_VERSION 1)
set (PNT_INTEGRITY_MINOR_VERSION 16)
set (PNT_INTEGRITY_PATCH_VERSION 1)

set (PROJECT_VERSION ${PNT_INTEGRITY_MAJOR_VERSION}.${PNT_INTEGRITY_MINOR_VERSION}.${PNT_INTEGRITY_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${PROJECT_VERSION}")

###############################################################################
## Set compiler options                                                      ##
###############################################################################
# suppress policy warnings on newer versions of cmake
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0053 NEW)
cmake_policy(SET CMP0057 NEW)

###############################################################################
## Find dependencies                                                         ##
###############################################################################
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config/)

find_package(FFTW QUIET)

if (${FFTW_FOUND})
  message(STATUS "Build acquisition check.")
  set(BUILD_ACQUISTION_CHECK TRUE)
else() 
  message(STATUS "Not building acquisition check.")
  set(BUILD_ACQUISTION_CHECK FALSE)
endif()

if(NOT DEFINED EIGEN3_INCLUDE_DIR)
  find_package(Eigen3 REQUIRED)
endif()

find_package(logutils REQUIRED)
find_package(if_utils REQUIRED)

###############################################################################
## Set source files                                                          ##
###############################################################################

# Add default source files
# Add default source files
set(PNT_INTEGRITY_SRCS src/AssuranceCheck.cpp
                       src/IntegrityMonitor.cpp
                       src/IntegrityDataRepository.cpp
                       src/RepositoryEntry.cpp
                       src/AngleOfArrivalCheck.cpp
                       src/RangePositionCheck.cpp
                       src/StaticPositionCheck.cpp
                       src/NavigationDataCheck.cpp
                       src/PositionVelocityConsistencyCheck.cpp
                       src/CnoCheck.cpp
                       src/PositionJumpCheck.cpp
                       src/AgcCheck.cpp
                       src/ClockBiasCheck.cpp
                       src/GPSNavDataCommon.cpp
                       src/GPSAlmanac.cpp
                       src/GPSEphemeris.cpp)
# Add default header files
set(PNT_INTEGRITY_HEADERS  include/pnt_integrity/AssuranceCheck.hpp
                           include/pnt_integrity/IntegrityData.hpp
                           include/pnt_integrity/IntegrityMonitor.hpp
                           include/pnt_integrity/IntegrityDataRepository.hpp
                           include/pnt_integrity/RepositoryEntry.hpp
                           include/pnt_integrity/AngleOfArrivalCheck.hpp
                           include/pnt_integrity/RangePositionCheck.hpp
                           include/pnt_integrity/StaticPositionCheck.hpp
                           include/pnt_integrity/NavigationDataCheck.hpp
                           include/pnt_integrity/CnoCheck.hpp
                           include/pnt_integrity/PositionJumpCheck.hpp
                           include/pnt_integrity/AgcCheck.hpp
                           include/pnt_integrity/ClockBiasCheck.hpp
                           include/pnt_integrity/GPSNavDataCommon.hpp
                           include/pnt_integrity/GPSAlmanac.hpp
                           include/pnt_integrity/GPSEphemeris.hpp)

if(BUILD_ACQUISTION_CHECK)
  list(APPEND PNT_INTEGRITY_SRCS src/AcquisitionCheck.cpp)
  list(APPEND PNT_INTEGRITY_HEADERS include/pnt_integrity/AcquisitionCheck.hpp)
endif()

###############################################################################
## Build C++ library                                                         ##
###############################################################################

## Declare a cpp library
add_library(${PROJECT_NAME} ${PNT_INTEGRITY_SRCS} ${PNT_INTEGRITY_HEADERS})

target_link_libraries(${PROJECT_NAME}
    PUBLIC
      logutils
      if_utils
      ${FFTW_LIBRARIES}
)

set(INCLUDES  $<INSTALL_INTERFACE:include>    
              $<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIR}>
              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
              $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
              )

if(BUILD_ACQUISTION_CHECK)
  list(APPEND INCLUDES 
              $<INSTALL_INTERFACE:${FFTW_INCLUDE_DIRS}> 
              $<BUILD_INTERFACE:${FFTW_INCLUDE_DIRS}>
              )
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        ${INCLUDES}
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
if(NOT MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wno-psabi>)

## Executables
option(BUILD_PNT_INTEGRITY_EXAMPLES "Build examples." OFF)
IF(BUILD_PNT_INTEGRITY_EXAMPLES)
  add_executable(repo_test_app examples/repoTestApp.cpp)
  target_link_libraries(repo_test_app
                        logutils
                        if_utils
                        ${PROJECT_NAME}
  )

  target_compile_features(repo_test_app PRIVATE cxx_std_14)
  target_compile_options(repo_test_app PRIVATE -Wall -Wextra -Wpedantic)

  if (BUILD_ACQUISTION_CHECK)
    add_executable(test_acquisition_check examples/testAcquisitionCheck.cpp)
    target_link_libraries(test_acquisition_check ${PROJECT_NAME})

    target_compile_features(test_acquisition_check PRIVATE cxx_std_14)
    target_compile_options(test_acquisition_check PRIVATE -Wall -Wextra -Wpedantic)
  endif()
ENDIF(BUILD_PNT_INTEGRITY_EXAMPLES)

set(PNT_INTEGRITY_INCLUDES_ACQ_CHECK ${BUILD_ACQUISTION_CHECK} PARENT_SCOPE) 

###############################################################################
## Install targets                                                           ##
###############################################################################

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
        EXPORT pnt_integrityTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
)

IF(BUILD_PNT_INTEGRITY_EXAMPLES)

  install(TARGETS repo_test_app DESTINATION bin)
  if(BUILD_ACQUISTION_CHECK)
    install(TARGETS test_acquisition_check DESTINATION bin)
  endif()
ENDIF(BUILD_PNT_INTEGRITY_EXAMPLES)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    pnt_integrityConfigVersion.cmake
    COMPATIBILITY AnyNewerVersion
)

export(TARGETS ${PROJECT_NAME} FILE pnt_integrityTargets.cmake)

install(EXPORT pnt_integrityTargets
  FILE pnt_integrityTargets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

set(PKG_NAME ${PROJECT_NAME})

configure_package_config_file("cmake/pnt_integrityConfig.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/pnt_integrityConfig.cmake"
                              INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
                              PATH_VARS PKG_NAME
                              NO_SET_AND_CHECK_MACRO
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/pnt_integrityConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/pnt_integrityConfigVersion.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME}
)
