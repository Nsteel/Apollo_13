qt4_wrap_cpp(MOC_HEADER_SOURCES
    DiscreteAnglesSpinBox.h
    GLWidget.h
    MotionPrimitiveDesignerWindow.h)

add_executable(unicycle
    unicycle.cpp
    ${MOC_HEADER_SOURCES}
    DiscreteAnglesSpinBox.cpp
    MotionPrimitiveDesignerWindow.cpp
    GLWidget.cpp
    unicycle_motions.cpp)

target_link_libraries(unicycle ${QT_LIBRARIES} ${OPENGL_LIBRARIES})
