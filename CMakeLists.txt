cmake_minimum_required(VERSION 3.16)
project(pogosim VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules" ${CMAKE_MODULE_PATH})

# Set C++ and C standards
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find required packages
find_package(Boost REQUIRED COMPONENTS system)
find_package(SDL2 REQUIRED)
find_package(SDL2_gfx REQUIRED)
find_package(SDL2_ttf REQUIRED)
#find_package(Box2D REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

# Collect source files
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.c"
)
list(FILTER SOURCES EXCLUDE REGEX "/(libs|tests|pogobot-sdk|examples)/.*$")

# Create library target
add_library(${PROJECT_NAME} ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${SDL2_INCLUDE_DIRS}
        ${SDL2_gfx_INCLUDE_DIRS}
        ${SDL2_ttf_INCLUDE_DIRS}
        #        ${BOX2D_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        ${spdlog_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Boost::system
        SDL2::SDL2
        #        Box2D::Box2D
        yaml-cpp
        spdlog::spdlog
)

# Set compile options
target_compile_options(${PROJECT_NAME}
    PRIVATE
        $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -Wunused-parameter -O3 -msse4.1 -mpclmul>
        $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# Install rules
include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Generate and install CMake config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)


###### BUILD EXAMPLES ######
# Find all subdirectories in examples
file(GLOB EXAMPLE_DIRS RELATIVE ${CMAKE_SOURCE_DIR}/examples ${CMAKE_SOURCE_DIR}/examples/*)

# Create a custom target that will build all examples
add_custom_target(examples ALL
    DEPENDS ${PROJECT_NAME}
    COMMENT "Building all examples"
)

# For each example directory
foreach(EXAMPLE ${EXAMPLE_DIRS})
    if(IS_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/${EXAMPLE})
        # Create a custom target for this specific example
        add_custom_target(example_${EXAMPLE}
            COMMAND ${CMAKE_COMMAND} -E echo "Building example: ${EXAMPLE}"
            COMMAND make clean
            COMMAND make sim
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/${EXAMPLE}
            DEPENDS ${PROJECT_NAME}
            COMMENT "Building example ${EXAMPLE}"
        )
        
        # Add this example target as a dependency of the main examples target
        add_dependencies(examples example_${EXAMPLE})
    endif()
endforeach()

