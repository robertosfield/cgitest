# chosen arbitrarily

cmake_minimum_required(VERSION 3.20)

project(cgitest LANGUAGES CXX)

find_package(OpenGL REQUIRED)

find_package(OpenSceneGraph 3.6.5 REQUIRED OpenThreads osgViewer osgText osgGA osgDB osgUtil)

find_path(IOSDEFN_INCLUDE_DIR SIM/iosdefn.h REQUIRED
    HINTS ${PROJECT_SOURCE_DIR})

add_executable(cgitest cgitest.cpp)


target_include_directories(cgitest SYSTEM PUBLIC
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIR}
    ${IOSDEFN_INCLUDE_DIR}
)

target_link_libraries(cgitest
    ${OPENTHREADS_LIBRARIES}
    ${OSGVIEWER_LIBRARIES}
    ${OSGTEXT_LIBRARIES}
    ${OSGGA_LIBRARIES}
    ${OSGDB_LIBRARIES}
    ${OSG_LIBRARIES}
    ${OSGUTIL_LIBRARIES}
    ${OPENGL_gl_LIBRARY}
)
