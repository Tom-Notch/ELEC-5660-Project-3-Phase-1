# ----------------------------------------------------------------------------
# Basic Configuration
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.6)

project(aruco)
set(PROJECT_VERSION "1.2.4")
string(REGEX MATCHALL "[0-9]" PROJECT_VERSION_PARTS "${PROJECT_VERSION}")
list(GET PROJECT_VERSION_PARTS 0 PROJECT_VERSION_MAJOR)
list(GET PROJECT_VERSION_PARTS 1 PROJECT_VERSION_MINOR)
list(GET PROJECT_VERSION_PARTS 2 PROJECT_VERSION_PATCH)
set(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")

# ------------------------------------------------------
# Build type
# ------------------------------------------------------

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# ------------------------------------------------------
# Lib Names and Dirs
# ------------------------------------------------------

if(WIN32)
  # Postfix of DLLs:
  set(PROJECT_DLLVERSION
      "${PROJECT_VERSION_MAJOR}${PROJECT_VERSION_MINOR}${PROJECT_VERSION_PATCH}"
  )
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}")
  set(RUNTIME_OUTPUT_PATH
      ${PROJECT_BINARY_DIR}/bin
      CACHE PATH "Directory for dlls and binaries")
  set(EXECUTABLE_OUTPUT_PATH
      ${PROJECT_BINARY_DIR}/bin
      CACHE PATH "Directory for binaries")
  set(LIBRARY_OUTPUT_PATH
      ${PROJECT_BINARY_DIR}/bin
      CACHE PATH "Directory for dlls")
else()
  # Postfix of so's:
  set(PROJECT_DLLVERSION)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/lib/cmake/
                        /usr/lib/cmake)
endif()

option(BUILD_SHARED_LIBS "Set to OFF to build static libraries" ON)

# ----------------------------------------------------------------------------
# Find Dependencies
# ----------------------------------------------------------------------------

find_package(OpenCV REQUIRED)
set(REQUIRED_LIBRARIES ${OpenCV_LIBS})

if(EXISTS ${GLUT_PATH})
  include_directories(${GLUT_PATH}/include)
  set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} ${GLUT_PATH}/lib)
endif()

# LOOK FOR OPENGL AND GLUT FIND OPENGL LIBRARY. In Android, there is no need
# since it is included
if(NOT ANDROID_CREATION)
  find_package(OpenGL)
  find_package(GLUT) # standard package

  message(
    STATUS
      "GLUT_FOUND=${GLUT_FOUND} OPENGL_gl_LIBRARY=${OPENGL_gl_LIBRARY} GLUT_HEADER=${GLUT_HEADER}"
  )
  if(NOT GLUT_FOUND) # else, freeglut
    find_library(GLUT_glut_LIBRARY NAMES freeglut)
    message(STATUS "GLUT_glut_LIBRARY=${GLUT_glut_LIBRARY}")
  endif()
  if((NOT GLUT_glut_LIBRARY AND NOT GLUT_FOUND) OR NOT OPENGL_gl_LIBRARY)
    set(GL_FOUND "NO")
  else()
    set(GL_FOUND "YES")
    set(OPENGL_LIBS ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
                    ${GLUT_glut_LIBRARY})
  endif()
endif()

# ----------------------------------------------------------------------------
# PROJECT CONFIGURATION force some variables that could be defined in the
# command line to be written to cache
# ----------------------------------------------------------------------------
option(INSTALL_DOC "Set to ON to build/install Documentation" OFF)
if(INSTALL_DOC)
  find_package(Doxygen REQUIRED)
  message(STATUS "INSTALL_DOC:         ${INSTALL_DOC} ")
  include("${PROJECT_SOURCE_DIR}/generateDoc.cmake")
  generate_documentation(${PROJECT_SOURCE_DIR}/dox.in)
endif()

# ----------------------------------------------------------------------------
# Uninstall target, for "make uninstall"
# ----------------------------------------------------------------------------
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
                            "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

# ----------------------------------------------------------------------------
# create configuration file from .in file (If you use windows take care with
# paths)
# ----------------------------------------------------------------------------

configure_file("${PROJECT_SOURCE_DIR}/config.cmake.in"
               "${PROJECT_BINARY_DIR}/Find${PROJECT_NAME}.cmake")
install(FILES "${PROJECT_BINARY_DIR}/Find${PROJECT_NAME}.cmake"
        DESTINATION lib/cmake/)

# ----------------------------------------------------------------------------
# Program Optimization and debug (Extracted from OpenCV)
# ----------------------------------------------------------------------------
set(WARNINGS_ARE_ERRORS
    OFF
    CACHE BOOL "Treat warnings as errors")
set(WHOLE_PROGRAM_OPTIMIZATION
    OFF
    CACHE BOOL "Flags for whole program optimization.")

set(EXTRA_C_FLAGS "")
set(EXTRA_C_FLAGS_RELEASE "")
set(EXTRA_C_FLAGS_DEBUG "")
set(EXTRA_EXE_LINKER_FLAGS "")
set(EXTRA_EXE_LINKER_FLAGS_RELEASE "")
set(EXTRA_EXE_LINKER_FLAGS_DEBUG "")

if(CMAKE_COMPILER_IS_GNUCXX OR MINGW)
  set(ENABLE_PROFILING
      OFF
      CACHE BOOL "Enable profiling in the GCC compiler (Add flags: -g -pg)")
  set(USE_OMIT_FRAME_POINTER
      ON
      CACHE BOOL "Enable -fomit-frame-pointer for GCC")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES arm*) # We can use only -O2 because the
                                             # -O3 causes gcc crash
    set(USE_O2
        ON
        CACHE BOOL "Enable -O2 for GCC")
    set(USE_FAST_MATH
        ON
        CACHE BOOL "Enable -ffast-math for GCC")
  endif()
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES powerpc*)
    set(USE_O3
        ON
        CACHE BOOL "Enable -O3 for GCC")
    set(USE_POWERPC
        ON
        CACHE BOOL "Enable PowerPC for GCC")
  endif()
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64* OR ${CMAKE_SYSTEM_PROCESSOR}
                                                 MATCHES x86_64*)
    set(USE_O3
        ON
        CACHE BOOL "Enable -O3 for GCC")
    set(USE_FAST_MATH
        ON
        CACHE BOOL "Enable -ffast-math for GCC")
    set(USE_MMX
        ON
        CACHE BOOL "Enable MMX for GCC")
    set(USE_SSE
        ON
        CACHE BOOL "Enable SSE for GCC")
    set(USE_SSE2
        ON
        CACHE BOOL "Enable SSE2 for GCC")
    set(USE_SSE3
        ON
        CACHE BOOL "Enable SSE3 for GCC")
  endif()
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES i686* OR ${CMAKE_SYSTEM_PROCESSOR}
                                                MATCHES x86)
    set(USE_O3
        ON
        CACHE BOOL "Enable -O3 for GCC")
    set(USE_FAST_MATH
        ON
        CACHE BOOL "Enable -ffast-math for GCC")
    set(USE_MMX
        ON
        CACHE BOOL "Enable MMX for GCC")
    set(USE_SSE
        OFF
        CACHE BOOL "Enable SSE for GCC")
    set(USE_SSE2
        OFF
        CACHE BOOL "Enable SSE2 for GCC")
    set(USE_SSE3
        OFF
        CACHE BOOL "Enable SSE3 for GCC")
  endif()

  set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -Wall")

  if(WARNINGS_ARE_ERRORS)
    set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -Werror")
  endif()

  # The -Wno-long-long is required in 64bit systems when including sytem
  # headers.
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86_64* OR ${CMAKE_SYSTEM_PROCESSOR}
                                                  MATCHES amd64*)
    set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -Wno-long-long")
  endif()

  # Whole program optimization
  if(WHOLE_PROGRAM_OPTIMIZATION)
    set(EXTRA_C_FLAGS_RELEASE
        "${EXTRA_C_FLAGS_RELEASE} -fwhole-program --combine")
  endif()

  # Other optimizations
  if(USE_OMIT_FRAME_POINTER)
    set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -fomit-frame-pointer")
  endif()
  if(USE_O2)
    set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -O2")
  endif()
  if(USE_O3)
    set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -O3")
  endif()
  if(USE_FAST_MATH)
    set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -ffast-math")
  endif()
  if(USE_POWERPC)
    set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -mcpu=G3 -mtune=G5")
  endif()
  if(USE_MMX)
    set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -mmmx")
  endif()
  if(USE_SSE)
    set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -msse")
  endif()
  if(USE_SSE2)
    set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -msse2")
  endif()
  if(USE_SSE3 AND NOT MINGW) # SSE3 should be disabled under MingW because it
                             # generates compiler errors
    set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -msse3")
  endif()

  if(ENABLE_PROFILING)
    set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -pg -g")
  else()
    if(NOT APPLE)
      set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS} -ffunction-sections")
    endif()
  endif()

  # Parallel mode
  if(ENABLE_OPENMP)
    set(EXTRA_C_FLAGS "${EXTRA_C_FLAGS}  -fopenmp")
    set(LINKER_LIBS ${LINKER_LIBS} gomp)
  endif()
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES armv7l) # In ARM_COrtex8 with neon,
                                               # enalble vectorized operations
    set(EXTRA_C_FLAGS_RELEASE
        "${EXTRA_C_FLAGS_RELEASE} -mcpu=cortex-a8 -mfpu=neon -mfloat-abi=softfp -ftree-vectorize "
    )
  endif()

  set(EXTRA_C_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE} -DNDEBUG")
  set(EXTRA_C_FLAGS_DEBUG
      "-g3 -O0 -DDEBUG -D_DEBUG -W -Wextra -Wno-return-type")

  message(
    STATUS
      "-------------------------------------------------------------------------------"
  )
  message(STATUS "GNU COMPILER")
  message(
    STATUS
      "-------------------------------------------------------------------------------"
  )

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_C_FLAGS}")
  set(CMAKE_CXX_FLAGS_RELEASE "${EXTRA_C_FLAGS_RELEASE}")
  set(CMAKE_CXX_FLAGS_DEBUG "${EXTRA_C_FLAGS_DEBUG}")
  set(CMAKE_EXE_LINKER_FLAGS
      "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_EXE_LINKER_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE
      "${CMAKE_EXE_LINKER_FLAGS_RELEASE} ${EXTRA_EXE_LINKER_FLAGS_RELEASE}")
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG
      "${CMAKE_EXE_LINKER_FLAGS_DEBUG} ${EXTRA_EXE_LINKER_FLAGS_DEBUG}")
endif()

# ----------------------------------------------------------------------------
# PACKAGE GENERATOR
# ----------------------------------------------------------------------------

if(UNIX)
  set(CPACK_GENERATOR "DEB")
  set(CPACK_PACKAGE_CONTACT "rmsalinas@uco.es")
  set(CPACK_DEBIAN_PACKAGE_MAINTAINER "rmsalinas@uco.es")
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "ArUco Library")
  set(CPACK_PACKAGE_VENDOR
      "Rafael Munoz-Salinas. AVA Group. University of Cordoba. Spain")
  set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
  set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
  set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
  set(CPACK_PACKAGE_INSTALL_DIRECTORY
      "CMake ${CMake_VERSION_MAJOR}.${CMake_VERSION_MINOR}")
  set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
  set(CPACK_STRIP_FILES "bin/lib${PROJECT_NAME}")
  set(CPACK_SOURCE_STRIP_FILES "")
  set(CPACK_PACKAGING_INSTALL_PREFIX "/usr/")
  if(CPACK_GENERATOR STREQUAL "DEB")
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "opencv (>= 2.1.0)")
  endif()
  include(CPack)
endif()

# ------------------------------------------------
# DIRS
# ------------------------------------------------
add_subdirectory(src)
if(NOT ANDROID_CREATION)
  add_subdirectory(utils)
endif()

# ----------------------------------------------------------------------------
# display status message for important variables
# ----------------------------------------------------------------------------
message(STATUS)
message(
  STATUS
    "-------------------------------------------------------------------------------"
)
message(STATUS "General configuration for ${PROJECT_NAME} ${PROJECT_VERSION}")
message(
  STATUS
    "-------------------------------------------------------------------------------"
)
message(STATUS)
message("    Built as dynamic libs?:" ${BUILD_SHARED_LIBS})
message("    Compiler:" "${CMAKE_COMPILER}" "${CMAKE_CXX_COMPILER}")
message(
  STATUS
    "C++ flags (Release):       ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
message(
  STATUS
    "C++ flags (Debug):         ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "CMAKE_CXX_FLAGS:         ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_BINARY_DIR:         ${CMAKE_BINARY_DIR}")

message(STATUS)
message(STATUS "CMAKE_SYSTEM_PROCESSOR = ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "BUILD_SHARED_LIBS = ${BUILD_SHARED_LIBS}")
message(STATUS "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")

message(STATUS)
message(
  STATUS
    "---------------------------     Documentation     -----------------------------"
)
message(STATUS)
message(STATUS "INSTALL_DOC = ${INSTALL_DOC}")
message(STATUS "USE_LATEX = ${USE_LATEX}")
message(STATUS "USE_DOT = ${USE_DOT}")
message(STATUS "USE_CHM = ${USE_CHM}")
message(STATUS)
message(STATUS "FOUND OPENGL=${GL_FOUND}    LIBS=${OPENGL_LIBS}")
message(STATUS "OpenCV_LIB_DIR=${OpenCV_LIB_DIR}")
message(STATUS "CMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}")
message(STATUS)
message(STATUS)
message(STATUS "Change a value with: cmake -D<Variable>=<Value>")
message(STATUS)
