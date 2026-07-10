cmake_minimum_required(VERSION 3.5.0)

set(PROJECT "stokesim")
project(${PROJECT})

################################################## OPTIONS

option(BUILD_DOXYGEN "Generate API documentation using Doxygen" ON)

option(RUN_CONFIG_GENS "Run each '.gen' executable in ./config/" ON)
option(VERBOSE_CONFIG_GEN "Enable verbose messages during config generation" OFF)

option(BUILD_LOGGER_PLUGIN "Build the example logger plugin in ./plugins/" ON)
option(BUILD_TRIGGER_PLUGIN "Build the example trigger plugin in ./plugins/" ON)

set(BUILD_ROS_PLUGIN OFF CACHE STRING "Build the example ROS plugin in ./plugins/")
if(DEFINED ENV{ROS_ROOT})
    set(BUILD_ROS_PLUGIN ON)
endif()

option(DEBUG_BUILD "Build with -O0 -g and verbose makefile" OFF)

set(INSTALL_PREFIX "/usr/local" CACHE STRING "Destination with subdirectories 'lib', 'include', and 'bin' to install files to")

################################################## READ-OUT

function(print_config_flag flag msg)
    if(${flag})
        message(STATUS "${msg} YES")
    else()
        message(STATUS "${msg} NO")
    endif()
endfunction(print_config_flag)

message(STATUS "=========================")
message(STATUS "= Configuration Options =")
message(STATUS "=========================")
print_config_flag(${BUILD_DOXYGEN}               "Make API documentation with Doxygen:  ")
print_config_flag(${RUN_CONFIG_GENS}             "Run each config-generation script:    ")
print_config_flag(${VERBOSE_CONFIG_GEN}          "Enable messages during config gen:    ")
print_config_flag(${BUILD_LOGGER_PLUGIN}         "Build the example logger plugin:      ")
print_config_flag(${BUILD_TRIGGER_PLUGIN}        "Build the example trigger plugin:     ")
print_config_flag(${BUILD_ROS_PLUGIN}            "Build the example ROS plugin:         ")
print_config_flag(${DEBUG_BUILD}                 "Build for debugging:                  ")
message(STATUS                                   "Prefix for 'make install' directories: ${INSTALL_PREFIX}")
message(STATUS "=========================")

################################################## COMPILER SPECIFICS

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -Wfatal-errors -Wunused-variable -Wreorder -Wall")
if(${DEBUG_BUILD})
    set(CMAKE_VERBOSE_MAKEFILE ON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
else()
    set(CMAKE_VERBOSE_MAKEFILE OFF)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -flto")
endif()

################################################## DIRECTORIES

# Find Eigen
foreach(DIR "/usr/local/include/eigen3"
            "/usr/include/eigen3")
    if(EXISTS ${DIR})
        set(EIGEN_INCLUDE_DIR ${DIR})
        break()
    endif()
endforeach(DIR)
if(NOT DEFINED EIGEN_INCLUDE_DIR)
    message(FATAL_ERROR "Eigen3 is not installed in any usual location!")
endif()

##

find_package("yaml-cpp" REQUIRED)

##

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include"
                    ${EIGEN_INCLUDE_DIR}
                    ${YAML_CPP_INCLUDE_DIR})

##

set(PLUGIN_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/plugins")
set(LOG_DIR "${CMAKE_CURRENT_BINARY_DIR}/logs")
file(MAKE_DIRECTORY ${LOG_DIR})

##

if(${RUN_CONFIG_GENS})
    set(CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/config")
endif()

################################################## SOURCES

file(GLOB MATH_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/source/math/*.cpp")
file(GLOB PHYSICS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/source/physics/*.cpp"
                          "${CMAKE_CURRENT_SOURCE_DIR}/source/physics/actuator_types/*.cpp"
                          "${CMAKE_CURRENT_SOURCE_DIR}/source/physics/sensor_types/*.cpp")
file(GLOB IO_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/source/io/*.cpp")

##

if(${BUILD_LOGGER_PLUGIN})
    file(GLOB LOGGER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/plugins/logger/logger.cpp")
endif()

##

if(${BUILD_TRIGGER_PLUGIN})
    file(GLOB TRIGGER_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/plugins/trigger/trigger.cpp")
endif()

##

if(${BUILD_ROS_PLUGIN})
    file(GLOB ROS_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/plugins/stokesros/stokesros.cpp")
endif()

##

if(${RUN_CONFIG_GENS})
    file(GLOB CONFIG_GENS "${CONFIG_DIR}/*.gen")
endif()

################################################## LIBRARY TARGETS

add_library(${PROJECT} SHARED "${CMAKE_CURRENT_SOURCE_DIR}/source/simulator.cpp"
                              ${MATH_SOURCES}
                              ${PHYSICS_SOURCES}
                              ${IO_SOURCES})
target_link_libraries(${PROJECT} ${CMAKE_DL_LIBS})
add_dependencies(${PROJECT} "yaml-cpp")

##

if(${BUILD_LOGGER_PLUGIN})
    add_library("logger" SHARED ${LOGGER_SOURCES})
    target_link_libraries("logger" ${PROJECT})
    set_target_properties("logger" PROPERTIES PREFIX ""
                                              LIBRARY_OUTPUT_DIRECTORY ${PLUGIN_BUILD_DIR})
endif()

##

if(${BUILD_TRIGGER_PLUGIN})
    add_library("trigger" SHARED ${TRIGGER_SOURCES})
    target_link_libraries("trigger" ${PROJECT})
    set_target_properties("trigger" PROPERTIES PREFIX ""
                                               LIBRARY_OUTPUT_DIRECTORY ${PLUGIN_BUILD_DIR})
endif()

##

if(${BUILD_ROS_PLUGIN})
    link_directories("$ENV{ROS_ROOT}/../../lib")
    include_directories("$ENV{ROS_ROOT}/../../include")
    find_package(catkin REQUIRED COMPONENTS roscpp tf)
    add_library(stokesros SHARED "plugins/stokesros/stokesros.cpp")
    target_link_libraries(stokesros ${PROJECT}
                                    ${catkin_LIBRARIES})
    set_target_properties(stokesros  PROPERTIES PREFIX ""
                                                LIBRARY_OUTPUT_DIRECTORY ${PLUGIN_BUILD_DIR})
    add_library(rosclock SHARED "plugins/stokesros/rosclock.cpp")
    target_link_libraries(rosclock ${PROJECT}
                                   ${catkin_LIBRARIES})
    set_target_properties(rosclock  PROPERTIES PREFIX ""
                                               LIBRARY_OUTPUT_DIRECTORY ${PLUGIN_BUILD_DIR})
endif()

################################################## EXECUTABLE TARGETS

add_executable("${PROJECT}_executable" "${CMAKE_CURRENT_SOURCE_DIR}/source/main.cpp")
target_link_libraries("${PROJECT}_executable" ${PROJECT}
                                              ${YAML_CPP_LIBRARIES})
set_target_properties("${PROJECT}_executable" PROPERTIES OUTPUT_NAME ${PROJECT})

################################################# CONFIGURATION SCRIPTS

if(${RUN_CONFIG_GENS})
    if (${VERBOSE_CONFIG_GEN})
        message(STATUS "Running sim config generation scripts...")
    endif()
    foreach(script ${CONFIG_GENS})
        if (${VERBOSE_CONFIG_GEN})
            message("====")
            message(STATUS "${script}")
            execute_process(COMMAND ${script}
                            RESULT_VARIABLE result)
            message(STATUS "RETURN VALUE: ${result}")
        else()
            execute_process(COMMAND ${script}
                            OUTPUT_VARIABLE null
                            RESULT_VARIABLE result)
            if (NOT "${result}" STREQUAL "0")
                message(WARN "Config generation ${script} failed!")
            else()
                message(STATUS "Ran config generation ${script}")
            endif()
        endif()
    endforeach(script)
    message("====")
endif()

################################################# INSTALLATION TARGETS

# library
set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY true)
install(TARGETS ${PROJECT}
        DESTINATION "${INSTALL_PREFIX}/lib")

# executable
install(TARGETS "${PROJECT}_executable"
        DESTINATION "${INSTALL_PREFIX}/bin")

# headers
install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/${PROJECT}"
        DESTINATION "${INSTALL_PREFIX}/include")

################################################## DOCUMENTATION

if(${BUILD_DOXYGEN})
    find_package("Doxygen" REQUIRED)
    set(DOXYGEN_IN "${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in")
    set(DOXYGEN_OUT "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT})
    add_custom_target(docs_doxygen ALL
                      COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
                      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                      COMMENT "Generating API documentation with Doxygen"
                      VERBATIM)
endif()
