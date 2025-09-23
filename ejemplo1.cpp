#include "ejemplo1.h"
#include <QDebug>
#include <QSlider>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>

ejemplo1::ejemplo1(): Ui_Counter()
{
    setupUi(this);

    // Configuración inicial
    counterValue = 0;
    countUp = true; // Empezamos con cuenta progresiva
    initialCountdownValue = 100; // Valor inicial para cuenta regresiva

    lcdNumber->display(counterValue);

    // Configurar el temporizador
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCounter()));
    timer->start(1000); // 1 segundo inicial

    // Conectar el botón STOP/START original
    connect(button, SIGNAL(clicked()), this, SLOT(doButton()));

    // --- CREAR WIDGETS ADICIONALES ---

    // Botón de RESET
    QPushButton *resetButton = new QPushButton("RESET", this);
    resetButton->setGeometry(80, 260, 251, 40);
    connect(resetButton, SIGNAL(clicked()), this, SLOT(resetCounter()));

    // Slider para cambiar período
    QSlider *periodSlider = new QSlider(Qt::Horizontal, this);
    periodSlider->setGeometry(80, 310, 251, 30);
    periodSlider->setRange(100, 5000); // de 100ms a 5 segundos
    periodSlider->setValue(1000); // valor inicial 1 segundo
    connect(periodSlider, SIGNAL(valueChanged(int)), this, SLOT(changePeriod(int)));

    // Etiqueta para mostrar el período
    QLabel *periodLabel = new QLabel("Período: 1000 ms", this);
    periodLabel->setGeometry(80, 340, 251, 20);

    // Botón para cambiar entre cuenta progresiva/regresiva
    QPushButton *directionButton = new QPushButton("MODO: PROGRESIVA", this);
    directionButton->setGeometry(80, 370, 251, 40);
    connect(directionButton, SIGNAL(clicked()), this, SLOT(toggleCountDirection()));

    show();
}

void ejemplo1::doButton()
{
    if (timer->isActive()) {
        timer->stop();
        button->setText("START");
        qDebug() << "Timer stopped";
    } else {
        timer->start();
        button->setText("STOP");
        qDebug() << "Timer started";
    }
}

void ejemplo1::updateCounter()
{
    if (countUp) {
        // Cuenta progresiva
        counterValue++;
    } else {
        // Cuenta regresiva
        if (counterValue > 0) {
            counterValue--;
        } else {
            // Si llega a 0, opcional: detener el timer o resetear
            timer->stop();
            button->setText("START");
            qDebug() << "Countdown finished!";
        }
    }

    lcdNumber->display(counterValue);
    qDebug() << "Counter:" << counterValue;
}

void ejemplo1::resetCounter()
{
    if (countUp) {
        counterValue = 0;
    } else {
        counterValue = initialCountdownValue;
    }
    lcdNumber->display(counterValue);
    qDebug() << "Counter reset to:" << counterValue;
}

void ejemplo1::changePeriod(int value)
{
    timer->setInterval(value);
    qDebug() << "Period changed to:" << value << "ms";

    // Actualizar etiqueta del período (necesitaríamos una variable para la QLabel)
    // periodLabel->setText(QString("Período: %1 ms").arg(value));
}

void ejemplo1::toggleCountDirection()
{
    countUp = !countUp;

    // Actualizar el texto del botón
    QPushButton *directionButton = findChild<QPushButton*>();
    if (directionButton) {
        directionButton->setText(countUp ? "MODO: PROGRESIVA" : "MODO: REGRESIVA");
    }

    // Resetear el contador al cambiar de modo
    resetCounter();

    qDebug() << "Count direction changed to:" << (countUp ? "PROGRESIVA" : "REGRESIVA");
}

