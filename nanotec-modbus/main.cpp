#include <QCoreApplication>
#include "MotorController.h"

int main(int argc, char *argv[])
{
    if(argc < 1)
        return -1;

    QCoreApplication a(argc, argv);
    QString mPortName(argv[1]);
    MotorController *mController;

    mController = new MotorController(mPortName);
    if(mController->connectDevice()) {
        mController->stopMotor();
        mController->initializeProfileVelocity();
    }

    if(mController->isConnected())
        mController->startMotor(100, 0);

    return a.exec();
}
