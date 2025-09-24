#include "ejemplo1.h"                     // include the header file for this class
#include <QDebug>                         // for printing debug messages to console
#include <QSlider>                        // for the QSlider widget
#include <QPushButton>                    // for QPushButton widget
#include <QLabel>                         // for QLabel widget
#include <QVBoxLayout>                    // for layout management (though not used here directly)

ejemplo1::ejemplo1(): Ui_Counter()        // constructor for the class ejemplo1, inheriting from Ui_Counter
{
    // setupUi(this) is generated from the .ui file
    // it creates all widgets (buttons, labels, lcdNumber, etc.)
    // and attaches them to 'this' object so we can access them
    setupUi(this);

    // initial configuration
    counterValue = 0;                     // start counter at 0
    countUp = true;                       // by default we count upwards
    initialCountdownValue = 100;          // default value if counting down

    lcdNumber->display(counterValue);     // show initial counter value on LCD widget

    // configure timer
    timer = new QTimer(this);             // create a QTimer, parented to this widget
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCounter()));  // call updateCounter() every timeout
    timer->start(1000);                   // start timer with 1000ms (1 second) interval

    // connect the original STOP/START button (from .ui)
    connect(button, SIGNAL(clicked()), this, SLOT(doButton()));

    // --- CREATE ADDITIONAL WIDGETS ---

    // create RESET button
    QPushButton *resetButton = new QPushButton("RESET", this);
    resetButton->setGeometry(80, 260, 251, 40);                   // set position and size
    connect(resetButton, SIGNAL(clicked()), this, SLOT(resetCounter()));  // link to resetCounter()

    // create slider to change timer period
    QSlider *periodSlider = new QSlider(Qt::Horizontal, this);    // horizontal slider
    periodSlider->setGeometry(80, 310, 251, 30);                  // set position and size
    periodSlider->setRange(100, 5000);                            // min 100ms, max 5000ms
    periodSlider->setValue(1000);                                 // default = 1000ms (1 second)
    connect(periodSlider, SIGNAL(valueChanged(int)), this, SLOT(changePeriod(int))); // link slider to changePeriod()

    // create label to show current period
    QLabel *periodLabel = new QLabel("Período: 1000 ms", this);
    periodLabel->setGeometry(80, 340, 251, 20);

    // create button to switch between up/down counting
    QPushButton *directionButton = new QPushButton("MODO: PROGRESIVA", this);
    directionButton->setGeometry(80, 370, 251, 40);  // position and size
    connect(directionButton, SIGNAL(clicked()), this, SLOT(toggleCountDirection())); // toggle mode when clicked

    show();    // display the main widget
}

void ejemplo1::doButton()
{
    if (timer->isActive()) {               // if the timer is running
        timer->stop();                     // stop it
        button->setText("START");          // change button text to START
        qDebug() << "Timer stopped";       // log message
    } else {                               // if the timer is stopped
        timer->start();                    // start it again (uses last interval)
        button->setText("STOP");           // change button text to STOP
        qDebug() << "Timer started";       // log message
    }
}

void ejemplo1::updateCounter()
{
    if (countUp) {                         // if counting up
        counterValue++;                    // increase counter
    } else {                               // if counting down
        if (counterValue > 0) {            // while counter > 0
            counterValue--;                // decrease counter
        } else {                           // if reached 0
            timer->stop();                 // stop the timer
            button->setText("START");      // reset button text
            qDebug() << "Countdown finished!"; // log message
        }
    }

    lcdNumber->display(counterValue);      // update LCD display
    qDebug() << "Counter:" << counterValue;// log current value
}

void ejemplo1::resetCounter()
{
    if (countUp) {                         // if counting up
        counterValue = 0;                  // reset to 0
    } else {                               // if counting down
        counterValue = initialCountdownValue; // reset to initial countdown value (100)
    }
    lcdNumber->display(counterValue);      // update LCD display
    qDebug() << "Counter reset to:" << counterValue; // log message
}

void ejemplo1::changePeriod(int value)
{
    timer->setInterval(value);             // set new interval from slider
    qDebug() << "Period changed to:" << value << "ms";

    // update label text (but currently commented out because label isn't stored as a member)
    // periodLabel->setText(QString("Período: %1 ms").arg(value));
}

void ejemplo1::toggleCountDirection()
{
    countUp = !countUp;                    // invert direction flag

    // find the direction button dynamically (not the best approach — ideally store it as a member)
    QPushButton *directionButton = findChild<QPushButton*>();
    if (directionButton) {                 // if found
        directionButton->setText(countUp ? "MODO: PROGRESIVA" : "MODO: REGRESIVA"); // update text
    }

    resetCounter();                        // reset counter when changing mode

    qDebug() << "Count direction changed to:" << (countUp ? "PROGRESIVA" : "REGRESIVA"); // log
}
