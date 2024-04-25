# -helper macro to add a "doc" target with CMake build system. and configure
# doxy.config.in to doxy.config
#
# target "doc" allows building the documentation with doxygen/dot on WIN32 and
# Linux Creates .chm windows help file if MS HTML help workshop (available from
# http://msdn.microsoft.com/workshop/author/htmlhelp) is installed with its DLLs
# in PATH.
#
# Please note, that the tools, e.g.: doxygen, dot, latex, dvips, makeindex,
# gswin32, etc. must be in path.
#
# Note about Visual Studio Projects: MSVS hast its own path environment which
# may differ from the shell. See "Menu Tools/Options/Projects/VC++ Directories"
# in VS 7.1
#
# author Jan Woetzel 2004-2006 www.mip.informatik.uni-kiel.de/~jw
#
# Modified by Luis Díaz 2009 http://plagatux.es

macro(GENERATE_DOCUMENTATION DOX_CONFIG_FILE)
  find_package(Doxygen)
  if(DOXYGEN_FOUND)
    # Define variables
    set(SRCDIR "${PROJECT_SOURCE_DIR}/src")
    set(TAGFILE "${PROJECT_BINARY_DIR}/doc/${PROJECT_NAME}.tag")

    if(USE_CHM AND WIN32)
      set(WIN_CHM "YES")
      set(CHM_FILE "${PROJECT_SOURCE_DIR}/doc/help.chm")
      set(BINARY_TOC "YES")
      set(TOC_EXPAND "YES")
    else()
      set(WIN_CHM "NO")
      set(BINARY_TOC "NO")
      set(TOC_EXPAND "NO")
    endif()

    if(USE_LATEX)
      set(GENERATE_PDF "YES")
      set(GENERATE_LATEX "YES")
      set(LATEXOUT "latex")
    else()
      set(GENERATE_PDF "NO")
      set(GENERATE_LATEX "NO")
    endif()

    if(NOT USE_DOT)
      set(DOXYGEN_DOT_FOUND "NO")
    endif()

    # click+jump in Emacs and Visual Studio (for doxy.config) (jw)
    if(CMAKE_BUILD_TOOL MATCHES "(msdev|devenv)")
      set(DOXY_WARN_FORMAT "\"$file($line) : $text \"")
    else(CMAKE_BUILD_TOOL MATCHES "(msdev|devenv)")
      set(DOXY_WARN_FORMAT "\"$file:$line: $text \"")
    endif(CMAKE_BUILD_TOOL MATCHES "(msdev|devenv)")

    # we need latex for doxygen because of the formulas
    find_package(LATEX)
    if(NOT LATEX_COMPILER)
      message(
        STATUS
          "latex command LATEX_COMPILER not found but usually required. You will probably get warnings and user inetraction on doxy run."
      )
    endif(NOT LATEX_COMPILER)
    if(NOT MAKEINDEX_COMPILER)
      message(
        STATUS
          "makeindex command MAKEINDEX_COMPILER not found but usually required."
      )
    endif(NOT MAKEINDEX_COMPILER)
    if(NOT DVIPS_CONVERTER)
      message(
        STATUS "dvips command DVIPS_CONVERTER not found but usually required.")
    endif(NOT DVIPS_CONVERTER)

    # Check config file
    if(EXISTS "${DOX_CONFIG_FILE}")
      configure_file(${DOX_CONFIG_FILE} ${CMAKE_CURRENT_BINARY_DIR}/doxy.config
                     @ONLY) # OUT-OF-PLACE LOCATION
      set(DOXY_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/doxy.config")
    else()
      message(
        SEND_ERROR
          "Please create configuration file for doxygen in ${CMAKE_CURRENT_SOURCE_DIR}"
      )
    endif()

    # Add target
    add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${DOXY_CONFIG})

    if(WIN32 AND GENERATE_WIN_CHM STREQUAL "YES")
      find_package(HTMLHelp)
      if(HTML_HELP_COMPILER)
        add_custom_target(winhelp ${HTML_HELP_COMPILER} ${HHP_FILE})
        add_dependencies(winhelp doc)
        if(EXISTS ${CHM_FILE})
          if(PROJECT_NAME)
            set(OUT "${PROJECT_NAME}")
          else()
            set(OUT "Documentation") # default
          endif()
          if(${PROJECT_NAME}_VERSION_MAJOR)
            set(OUT "${OUT}-${${PROJECT_NAME}_VERSION_MAJOR}")
            if(${PROJECT_NAME}_VERSION_MINOR)
              set(OUT "${OUT}.${${PROJECT_NAME}_VERSION_MINOR}")
              if(${PROJECT_NAME}_VERSION_PATCH)
                set(OUT "${OUT}.${${PROJECT_NAME}_VERSION_PATCH}")
              endif()
            endif()
          endif()
          set(OUT "${OUT}.chm")
          install(
            FILES ${CHM_FILE}
            DESTINATION "doc"
            RENAME "${OUT}")
        endif()
      else()
        message(FATAL_ERROR "You have not Microsoft Help Compiler")
      endif()
    endif()

    install(DIRECTORY "${PROJECT_BINARY_DIR}/html/"
            DESTINATION "share/doc/lib${PROJECT_NAME}")

  endif(DOXYGEN_FOUND)
endmacro(GENERATE_DOCUMENTATION)
