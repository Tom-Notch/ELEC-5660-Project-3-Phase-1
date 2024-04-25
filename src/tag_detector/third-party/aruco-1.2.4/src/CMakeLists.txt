include_directories(.)
file(GLOB hdrs "*.h*")
file(GLOB srcs "*.c*")

set(hdrs ${hdrs})
set(srcs ${srcs})

add_library(${PROJECT_NAME} ${srcs} ${hdrs})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
message(STATUS "OpenCV_LIB_DIR=${OpenCV_LIB_DIR}")
set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES # create *nix style library versions + symbolic links
             DEFINE_SYMBOL DSO_EXPORTS
             VERSION ${PROJECT_VERSION}
             SOVERSION ${PROJECT_SOVERSION}
             CLEAN_DIRECT_OUTPUT 1 # allow creating static and shared libs
                                   # without conflicts
             OUTPUT_NAME
             "${PROJECT_NAME}${PROJECT_DLLVERSION}" # avoid conflicts between
                                                    # library and binary target
                                                    # names
)

target_link_libraries(${PROJECT_NAME} ${REQUIRED_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin COMPONENT main # Install the dll file in bin directory
  LIBRARY
    DESTINATION lib
    PERMISSIONS
      OWNER_READ
      OWNER_WRITE
      OWNER_EXECUTE
      GROUP_READ
      GROUP_EXECUTE
      WORLD_READ
      WORLD_EXECUTE
    COMPONENT main
  ARCHIVE DESTINATION lib COMPONENT main) # Install the dll.a file in lib
                                          # directory

install(
  FILES ${hdrs}
  DESTINATION include/${PROJECT_NAME}
  COMPONENT main)
