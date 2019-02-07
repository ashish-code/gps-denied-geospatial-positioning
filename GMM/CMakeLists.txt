cmake_minimum_required(VERSION 2.6)

#SET(ENV{BOOST_ROOT} "/opt/local/include")

#substitute your version number
find_package(Boost 1.41 REQUIRED COMPONENTS serialization-mt program_options python)
find_package(PythonLibs 2.6 REQUIRED)

set(QT_USE_QTSVG TRUE)
set(QT_USE_QTXML TRUE)
set(QT_USE_QTOPENGL TRUE)
find_package(OpenGL)
find_package( Qt4 REQUIRED )

INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}")
INCLUDE_DIRECTORIES("${PYTHON_INCLUDE_PATH}")
INCLUDE_DIRECTORIES(${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR} ${QT_QTSVG_INCLUDE_DIR}
                    ${QT_QTXML_INCLUDE_DIR} ${QT_QTOPENGL_INCLUDE_DIR} ${OPENGL_INCLUDE_DIRS})

QT4_WRAP_UI(osm_convert_FORMS mapviewer.ui)
QT4_WRAP_CPP(osm_convert_MOCS mapviewer.h mapwidget.h)
ADD_EXECUTABLE(osm_convert ${osm_convert_FORMS} ${osm_convert_MOCS} osm.cpp osm_convert.cpp mapviewer.cpp mapwidget.cpp objects.cpp coordinates.cpp utils.cpp)
TARGET_LINK_LIBRARIES(osm_convert ${Boost_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTSVG_LIBRARY}
                                  ${QT_QTXML_LIBRARY} ${QT_QTOPENGL_LIBRARY} ${OPENGL_LIBRARIES})

PYTHON_ADD_MODULE(osm_objects SHARED bindings.cpp objects.cpp coordinates.cpp utils.cpp)
TARGET_LINK_LIBRARIES(osm_objects ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})


