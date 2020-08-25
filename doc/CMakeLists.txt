# Copyright: (C) 2017 Fondazione Istituto Italiano di Tecnologia
# Authors: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

find_package(Doxygen)

if(DOXYGEN_FOUND)
    set(DOX_GENERATE_HTML YES)
    set(DOX_PATTERNS "*.h *.dox *.cpp *.cc *.hpp *.hh *.md")

    # search dot program
    # for setting the DOT_PATH variable in the Doxyfile
    find_program(DOT_EXECUTABLE dot)

    if( DOT_EXECUTABLE STREQUAL "DOT_EXECUTABLE-NOTFOUND" )
        message(STATUS "dot program not found")
        set(DOT_PATH "")
    else()
        string(REGEX REPLACE "dot$" "" DOT_PATH ${DOT_EXECUTABLE})
    endif()

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in
                   ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    add_custom_target(dox COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
                          WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()
