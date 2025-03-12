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
  src/physics/forces/Force.cpp
  src/physics/integrator.cpp
  src/physics/collision/ColDetector.cpp
  src/physics/collision/ColResponse.cpp
  src/physics/collision/Collision.cpp
  src/utils/geo/geoUtils.cpp
  src/utils/geo/geoConvex.cpp
  src/utils/DCEL/DCEL.cpp
  src/utils/DCEL/Vertex.cpp
  src/utils/DCEL/HalfEdge.cpp
  src/utils/DCEL/Face.cpp
)

# Specify where the include and library directories are for SFML
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../sfml3/source/include/)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include/)

# Set data directory
set(DATA_DIR ${CMAKE_BINARY_DIR}/data) # Specify directory variable
# Add custom command to always run
add_custom_target(CopyFonts ALL
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${CMAKE_SOURCE_DIR}/data/TTF ${DATA_DIR}/TTF
)
# Add final dependency to build file
add_dependencies(${PROJECT_NAME} CopyFonts)

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
  freetype
)

# Because I build statically
# add_compile_definitions(SFML_STATIC)

############################################################

if (CMAKE_BUILD_TYPE STREQUAL "Release")
  # Make a library that can be used outside the project
  add_library(${PROJECT_NAME}_lib SHARED
    src/ui/frame/SFMLWindow.cpp
    src/ui/frame/SFMLRenderer.cpp
    src/env/Environment.cpp
    src/env/bodies/Polygon.cpp
    src/env/bodies/Square.cpp
    src/physics/PhysicsEngine.cpp
    src/physics/forces/Force.cpp
    src/physics/integrator.cpp
    src/physics/collision/ColDetector.cpp
    src/physics/collision/ColResponse.cpp
    src/physics/collision/Collision.cpp
    src/utils/geo/geoUtils.cpp
    src/utils/geo/geoConvex.cpp
    src/utils/DCEL/DCEL.cpp
    src/utils/DCEL/Vertex.cpp
    src/utils/DCEL/HalfEdge.cpp
    src/utils/DCEL/Face.cpp
  )

  # Specify where the include and library directories are for SFML
  target_include_directories(${PROJECT_NAME}_lib PRIVATE ${CMAKE_SOURCE_DIR}/../sfml3/source/include/)
  target_include_directories(${PROJECT_NAME}_lib PUBLIC ${CMAKE_SOURCE_DIR}/include/)

  # Add final dependency to build file
  add_dependencies(${PROJECT_NAME}_lib CopyFonts)

  # Link to specific libraries depending on compile mode (release / debug)
  # Make it default to debug
  target_link_directories(${PROJECT_NAME}_lib PRIVATE ${CMAKE_SOURCE_DIR}/../sfml3/build/releaseShared/lib/)
  target_link_libraries(${PROJECT_NAME}_lib PRIVATE
    sfml-graphics
    sfml-window
    sfml-system
    sfml-network
    sfml-audio
  )

  # Necessary for sfml to work
  target_link_libraries(${PROJECT_NAME}_lib PRIVATE
    X11
    Xrandr
    Xcursor
    Xinerama
    Xi
    udev
    GL
    freetype
  )
endif()
message(STATUS "CXX Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
