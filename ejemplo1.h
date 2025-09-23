#ifndef ejemplo1_H
#define ejemplo1_H

#include <QtGui>
#include <QTimer>
#include "ui_counterDlg.h"

class ejemplo1 : public QWidget, public Ui_Counter
{
    Q_OBJECT
    public:
    ejemplo1();

public slots:
    void doButton();
    void updateCounter();
    void resetCounter();       // Nuevo slot para resetear
    void changePeriod(int value); // Nuevo slot para cambiar período
    void toggleCountDirection(); // Para cambiar entre cuenta progresiva/regresiva

private:
    int counterValue;
    QTimer *timer;
    bool countUp; // true = cuenta progresiva, false = cuenta regresiva
    int initialCountdownValue; // Valor inicial para cuenta regresiva
};

#endif // ejemplo1_H