#ifndef ejemplo1_H                    // header guard start: prevents multiple inclusions
#define ejemplo1_H

#include <QtGui>                      // include Qt GUI classes (widgets, layouts, etc.)
#include <QTimer>                     // include QTimer class for timed events
#include "ui_counterDlg.h"            // include the auto-generated UI class from Qt Designer

// declare the ejemplo1 class
class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT                          // Qt macro: enables signals/slots and other Qt features

    public:
    ejemplo1();                       // constructor declaration

public slots:                         // declare slots (functions that respond to signals)
    void doButton();                  // slot connected to start/stop button
    void updateCounter();             // slot called every timer timeout (updates counter)
    void resetCounter();              // slot to reset the counter to 0 or initial value
    void changePeriod(int value);     // slot to change timer interval from slider
    void toggleCountDirection();      // slot to toggle counting direction (up/down)

private:                              // private member variables
    int counterValue;                 // holds current counter value
    QTimer *timer;                    // pointer to QTimer object for timed events
    bool countUp;                     // true = count upwards, false = count downwards
    int initialCountdownValue;        // starting value when counting down
};

#endif // ejemplo1_H                   // end of header guard
