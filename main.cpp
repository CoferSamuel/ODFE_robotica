#include <QtWidgets>          // include all Qt widget classes (QApplication, QWidget, etc.)
#include "ejemplo1.h"         // include our custom widget class ejemplo1

int main(int argc, char** argv)  // standard C++ main function (argc = argument count, argv = argument values)
{
    QApplication app(argc, argv); // create the Qt application object, required for any GUI app
    ejemplo1 foo;                 // create an instance of our ejemplo1 widget (the main window)
    foo.show();                   // show the window on screen
    return app.exec();            // enter the Qt event loop, keeps the app running until it’s closed
}
