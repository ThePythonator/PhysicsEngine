cmake_minimum_required(VERSION 3.14)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Uncomment if you need to do some debugging :)
# set(CMAKE_FIND_DEBUG_MODE TRUE)

set(PROJECT_NAME PhysicsEngine)

# Add your sources here (adding headers is optional, but helps some CMake generators)
set(ENGINE_SOURCE
	"Collisions.cpp"
	"Constraints.cpp"
	"Shape.cpp"
	"RigidBody.cpp"

	"PhysicsEngineMath.cpp"

	"PhysicsEngine.cpp"
)


set(FRAMEWORK_SOURCE
	"BaseGame.cpp"
    "SDL2Extras.cpp"
    "Colour.cpp"
    "Input.cpp"
)

set(DEMO_SOURCE
	"Demo.cpp"
    "Game.cpp"
    "Constants.cpp"
)

# Uncomment if no console should be created
# set(WINDOWS_NO_CONSOLE)

project(${PROJECT_NAME})

list(TRANSFORM ENGINE_SOURCE PREPEND src/PhysicsEngine/)
list(TRANSFORM FRAMEWORK_SOURCE PREPEND src/Framework/)
list(TRANSFORM DEMO_SOURCE PREPEND src/Demo/)

set(PROJECT_SOURCES ${ENGINE_SOURCE};${FRAMEWORK_SOURCE};${DEMO_SOURCE})

set(PROJECT_DISTRIBS LICENSE README.md)

set(CONSOLE_FLAG)
if (WINDOWS_NO_CONSOLE)
	set(CONSOLE_FLAG WIN32)
endif()

add_executable(${PROJECT_NAME} ${CONSOLE_FLAG} MACOSX_BUNDLE ${PROJECT_SOURCES})

include(get_sdl2.cmake)

#target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

#include_directories(${PROJECT_SOURCE_DIR}/include/PhysicsEngine)
#include_directories(${PROJECT_SOURCE_DIR}/include/External)
#include_directories(${PROJECT_SOURCE_DIR}/include/Demo)

include_directories(${PROJECT_SOURCE_DIR}/include/Demo)
include_directories(${PROJECT_SOURCE_DIR}/include/External)
include_directories(${PROJECT_SOURCE_DIR}/include/Framework)
include_directories(${PROJECT_SOURCE_DIR}/include/PhysicsEngine)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Link
target_link_libraries(${PROJECT_NAME} SDL2::SDL2main SDL2::SDL2 SDL2::image)

# Setup release packages
install(TARGETS ${PROJECT_NAME}
	RUNTIME DESTINATION bin
	BUNDLE DESTINATION bin
)

install(FILES ${PROJECT_DISTRIBS}
	DESTINATION .
)

if (NOT EMSCRIPTEN)
    # Install dependencies
    set(DEP_SEARCH_DIRS)
    # We built these, so we're installing them
    if(TARGET SDL2)
        list(APPEND DEP_SEARCH_DIRS $<TARGET_FILE_DIR:SDL2>)
    endif()

    if(TARGET SDL2_image)
        list(APPEND DEP_SEARCH_DIRS $<TARGET_FILE_DIR:SDL2_image> $<TARGET_FILE_DIR:png> $<TARGET_FILE_DIR:zlib>)
    endif()

    set(EXE_SUFFIX)
    if(APPLE)
        set(EXE_SUFFIX ".app")
    endif()

    install(CODE "
        include(BundleUtilities)
        fixup_bundle(\"\${CMAKE_INSTALL_PREFIX}/bin/$<TARGET_FILE_NAME:${PROJECT_NAME}>${EXE_SUFFIX}\" \"\" \"${DEP_SEARCH_DIRS}\")
    ")
endif()

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)