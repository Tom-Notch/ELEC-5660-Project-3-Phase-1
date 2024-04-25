include_directories(${PROJECT_SOURCE_DIR}/src ${GNULIBS_INCLUDE_DIR})
link_libraries(${PROJECT_NAME} ${REQUIRED_LIBRARIES})

add_executable(aruco_test aruco_test.cpp)
add_executable(aruco_simple aruco_simple.cpp)
add_executable(aruco_create_marker aruco_create_marker.cpp)
add_executable(aruco_create_board aruco_create_board.cpp)
add_executable(aruco_selectoptimalmarkers aruco_selectoptimalmarkers.cpp)
add_executable(aruco_simple_board aruco_simple_board.cpp)
add_executable(aruco_test_board aruco_test_board.cpp)
add_executable(aruco_board_pix2meters aruco_board_pix2meters.cpp)
# ADD_EXECUTABLE(aruco_test_board_stability aruco_test_board_stability.cpp)

# INSTALL(TARGETS aruco_test aruco_simple aruco_create_marker RUNTIME
# DESTINATION bin)

install(
  TARGETS aruco_test
          aruco_board_pix2meters
          aruco_simple
          aruco_create_marker
          aruco_create_board
          aruco_simple_board
          aruco_test_board
          aruco_selectoptimalmarkers
  RUNTIME DESTINATION bin)
if(GL_FOUND)
  add_executable(aruco_test_gl aruco_test_gl.cpp)
  target_link_libraries(aruco_test_gl ${OPENGL_LIBS})

  add_executable(aruco_test_board_gl aruco_test_board_gl.cpp)
  target_link_libraries(aruco_test_board_gl ${OPENGL_LIBS})

  install(TARGETS aruco_test_gl aruco_test_board_gl RUNTIME DESTINATION bin)
endif(GL_FOUND)
