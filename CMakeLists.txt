# Make json config file
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Specify minimum required version
cmake_minimum_required(VERSION 3.31.4)

# Project name
project(SIS)

# Adds the main executable
add_executable(${PROJECT_NAME}
  src/main/Main.cpp
  src/ui/frame/SFMLWindow.cpp
  src/ui/frame/SFMLRenderer.cpp
  src/env/Environment.cpp
  src/env/bodies/Polygon.cpp
  src/env/bodies/Square.cpp
  src/physics/PhysicsEngine.cpp
  src/physics/integrator.cpp
)

# Specify where the include and library directories are for SFML
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../sfml3/source/include/)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include/)

# Link to specific libraries depending on compile mode (release / debug)
# Make it default to debug
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT EXISTS CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build." FORCE)
  endif()
endif()
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Debug mode (Use -DCMAKE_BUILD_TYPE=Release flag for release mode)")
  target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../sfml3/build/debug/lib/)
  target_link_libraries(${PROJECT_NAME} PRIVATE
    sfml-graphics-s-d
    sfml-window-s-d
    sfml-system-s-d
    sfml-network-s-d
    sfml-audio-s-d
  )
elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
  message(STATUS "Release mode")
  target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../sfml3/build/release/lib/)
  target_link_libraries(${PROJECT_NAME} PRIVATE
    sfml-graphics-s
    sfml-window-s
    sfml-system-s
    sfml-network-s
    sfml-audio-s
  )
endif()

# Necessary for sfml to work
target_link_libraries(${PROJECT_NAME} PRIVATE
  X11
  Xrandr
  Xcursor
  Xinerama
  Xi
  udev
  GL
)

# Because I build statically
add_compile_definitions(SFML_STATIC)
