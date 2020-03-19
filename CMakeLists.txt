cmake_minimum_required(VERSION 3.8)
project(Assignment3)

#-------------------------------------------------------------------------------
# Build options
#-------------------------------------------------------------------------------
option(BUILD_DEBUG     "Build with debug settings"    OFF)
option(BUILD_DOCS      "Build documentation"          OFF)
option(BUILD_CUSTOM    "Build without reference"      OFF)

if (MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D__SOLUTIONS__=50")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__SOLUTIONS__=50")
endif()

set(BUILD_DEBUG ${BUILD_DEBUG} CACHE BOOL "Build debug" FORCE)

if (BUILD_DEBUG)
  set(CMAKE_BUILD_TYPE Debug)
endif()

#-------------------------------------------------------------------------------
# Set target
#-------------------------------------------------------------------------------
set(APPLICATION_3_1_SOURCE

    # Scene Object & Structure
    src/scene/sphere.cpp
    src/scene/triangle.cpp
    src/scene/light.cpp
    src/scene/bvh.cpp
    src/scene/bbox.cpp

    # Pathtracer
    src/pathtracer/camera.cpp
    src/pathtracer/bsdf.cpp
    src/pathtracer/pathtracer.cpp
)

set(APPLICATION_3_2_SOURCE
    src/scene/object.cpp

    # Collada Parser
    src/scene/collada/collada.cpp
    src/scene/collada/camera_info.cpp
    src/scene/collada/light_info.cpp
    src/scene/collada/sphere_info.cpp
    src/scene/collada/polymesh_info.cpp
    src/scene/collada/material_info.cpp

    # Dynamic Scene
    src/scene/gl_scene/mesh.cpp
    src/scene/gl_scene/scene.cpp
    src/scene/gl_scene/sphere.cpp

    # MeshEdit
    src/util/halfEdgeMesh.cpp
    src/application/meshEdit.cpp

    src/pathtracer/sampler.cpp
    src/pathtracer/advanced_bsdf.cpp
    src/scene/environment_light.cpp
    src/pathtracer/camera_lens.cpp
    src/pathtracer/raytraced_renderer.cpp

    # misc
    src/util/sphere_drawing.cpp
    src/util/lodepng.cpp

    # Application
    src/application/application.cpp
    src/application/main.cpp
)

set(APPLICATION_HEADERS
    # Collada Parser
    src/scene/collada/camera_info.h
    src/scene/collada/collada_info.h
    src/scene/collada/collada.h
    src/scene/collada/light_info.h
    src/scene/collada/material_info.h
    src/scene/collada/polymesh_info.h
    src/scene/collada/sphere_info.h
    # Dynamic Scene
    src/scene/gl_scene/ambient_light.h
    src/scene/gl_scene/area_light.h
    src/scene/gl_scene/directional_light.h
    src/scene/gl_scene/draw_style.h
    src/scene/gl_scene/environment_light.h
    src/scene/gl_scene/material.h
    src/scene/gl_scene/mesh_view.h
    src/scene/gl_scene/mesh.h
    src/scene/gl_scene/point_light.h
    src/scene/gl_scene/scene.h
    src/scene/gl_scene/sphere.h
    src/scene/gl_scene/spot_light.h
    # Scene Object & Structure
    src/scene/aggregate.h
    src/scene/bbox.h
    src/scene/bvh.h
    src/scene/environment_light.h
    src/scene/light.h
    src/scene/object.h
    src/scene/primitive.h
    src/scene/scene.h
    src/scene/sphere.h
    src/scene/triangle.h
    # MeshEdit
    src/util/halfEdgeMesh.h
    src/util/image.h
    src/util/mutablePriorityQueue.h
    src/util/random_util.h
    src/util/work_queue.h
    # Pathtracer
    src/pathtracer/bsdf.h
    src/pathtracer/camera.h
    src/pathtracer/intersection.h
    src/pathtracer/pathtracer.h
    src/pathtracer/ray.h
    src/pathtracer/raytraced_renderer.h
    src/pathtracer/sampler.h
    # misc
    src/util/sphere_drawing.h
    src/util/lodepng.h
    # Application
    src/application/application.h
    src/application/meshEdit.h
    src/application/renderer.h
)

if (BUILD_CUSTOM)
  add_executable(pathtracer ${APPLICATION_3_1_SOURCE} ${APPLICATION_3_2_SOURCE} ${APPLICATION_HEADERS})
else()
  add_executable(pathtracer ${APPLICATION_3_2_SOURCE} ${APPLICATION_HEADERS})
  target_link_libraries(pathtracer PUBLIC pt31)

  include(CGL/find_avx.cmake)
  CHECK_FOR_AVX()

  if(HAVE_AVX2_EXTENSIONS)
    set(ARCHPREFIX libs/AVX2)
  elseif(HAVE_AVX_EXTENSIONS)
    set(ARCHPREFIX libs/AVX)
  else()
    set(ARCHPREFIX libs/SSE)
  endif()

  if (WIN32)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
      message(FATAL_ERROR "You should use Release profile on Windows, if you want to debug, set the configuration type to RelWithDebInfo in CMake settings")
    else()
      target_link_directories(pathtracer PRIVATE ${ARCHPREFIX}/x64_msvc/release)
    endif()
  elseif (APPLE)
    target_link_directories(pathtracer PRIVATE ${ARCHPREFIX}/x64_apple)
  elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_directories(pathtracer PRIVATE ${ARCHPREFIX}/x64_linux)
  else()
    message(FATAL_ERROR "Unknown system!")
  endif()

  target_link_libraries(pathtracer PRIVATE pt31)
endif()

target_include_directories(pathtracer PUBLIC src)

set(CGL_INCLUDE_DIRS CGL/include CGL/deps/glew/include CGL/deps/glfw/include)

#-------------------------------------------------------------------------------
# Find dependencies
#-------------------------------------------------------------------------------
add_subdirectory(CGL)
target_include_directories(pathtracer PUBLIC ${CGL_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CGL_CXX_FLAGS}")

set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)
target_link_libraries(pathtracer PUBLIC OpenGL::GL)
target_link_libraries(pathtracer PUBLIC OpenGL::GLU)

target_link_libraries(pathtracer PRIVATE CGL)

#-------------------------------------------------------------------------------
# Add subdirectories
#-------------------------------------------------------------------------------

# build documentation
if(BUILD_DOCS)
  find_package(DOXYGEN)
  if(DOXYGEN_FOUND AND BUILD_DOCS)
    add_subdirectory(docs)
  endif()
endif()

# Install settings
set(CMAKE_INSTALL_PREFIX "${Assignment1_SOURCE_DIR}/")
