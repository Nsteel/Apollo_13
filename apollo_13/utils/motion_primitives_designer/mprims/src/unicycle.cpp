#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include <QtOpenGL>
#include <QtGui>
#include "GLWidget.h"
#include "Pose2.h"
#include "MotionPrimitiveDesignerWindow.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MotionPrimitiveDesignerWindow main_window;
    main_window.show();

    return app.exec();
}
