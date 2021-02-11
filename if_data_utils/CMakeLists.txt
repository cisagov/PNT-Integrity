cmake_minimum_required(VERSION 2.8.11)
cmake_policy(SET CMP0048 NEW)
project(if_utils VERSION 1.3.2)

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
if(NOT DEFINED EIGEN3_INCLUDE_DIR)
  find_package(Eigen3 REQUIRED)
endif()


###############################################################################
## Set source files                                                          ##
###############################################################################

# Add default header files
set(IF_UTILS_HEADERS  include/if_data_utils/dataFileStructures.hpp
                      include/if_data_utils/IfData.hpp
                      include/if_data_utils/FileMux.hpp
                      include/if_data_utils/UpConvert.hpp
                      include/if_data_utils/IniReader.hpp
                      include/if_data_utils/IFDataFileReader.hpp
                      include/if_data_utils/IFDataFileWriter.hpp
                      include/if_data_utils/ini.h
                      include/if_data_utils/IFSampleData.hpp
)

# Add default source files
set(IF_UTILS_SRCS src/IfData.cpp
                  src/FileMux.cpp
                  src/UpConvert.cpp
                  src/IniReader.cpp
                  src/ini.c
)


###############################################################################
## Build C++ library                                                         ##
###############################################################################

## Declare a cpp library
add_library(${PROJECT_NAME} ${IF_UTILS_SRCS} ${IF_UTILS_HEADERS})

target_link_libraries(${PROJECT_NAME} 
    PUBLIC 
)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<INSTALL_INTERFACE:include>    
        $<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
)

###############################################################################
## Build C++ executables                                                     ##
###############################################################################
if(NOT BUILD_ZYNQ)
  add_executable(ifUtil examples/test.cpp)
  target_link_libraries(ifUtil ${PROJECT_NAME})
  target_compile_features(ifUtil PRIVATE cxx_std_11)

  add_executable(upConvert examples/upconvert.cpp)
  target_link_libraries(upConvert ${PROJECT_NAME} )
  target_compile_features(upConvert PRIVATE cxx_std_11)

  add_executable(combineFiles examples/combineFiles.cpp)    
  target_link_libraries(combineFiles ${PROJECT_NAME} ) 
  target_compile_features(combineFiles PRIVATE cxx_std_11)
endif()

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)
if(NOT MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()


###############################################################################
## Install targets                                                           ##
###############################################################################

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
        EXPORT if_utilsTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
)

if(NOT BUILD_ZYNQ)
  install(TARGETS ifUtil DESTINATION bin)
  install(TARGETS upConvert DESTINATION bin)
  install(TARGETS combineFiles DESTINATION bin)
endif()

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    if_utilsConfigVersion.cmake
    COMPATIBILITY AnyNewerVersion
)

export(TARGETS ${PROJECT_NAME} FILE if_utilsTargets.cmake)

install(EXPORT if_utilsTargets
  FILE if_utilsTargets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

set(PKG_NAME ${PROJECT_NAME})

configure_package_config_file("cmake/if_utilsConfig.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/if_utilsConfig.cmake"
                              INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
                              PATH_VARS PKG_NAME
                              NO_SET_AND_CHECK_MACRO
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/if_utilsConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/if_utilsConfigVersion.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME}
)
