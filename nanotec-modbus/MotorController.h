#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "MotorModbusMaster.h"

#include <QObject>

enum PowerStates{
    SwitchedOff = 0x00,
    Ready       = 0x01,
    SwitchedOn  = 0x02,
    Running     = 0x03,
    QuickStop   = 0x04,
    Fault       = 0x05,
    Unknown     = 0x06
};

enum MotorStates {
    Start   = 0x00,
    Stop    = 0x01,
    Pause   = 0x02,
    Init
};

class MotorController : public QObject
{
    Q_OBJECT
public:
    MotorController(QString port, QObject *parent = nullptr);
    ~MotorController();


    bool connectDevice();
    bool isConnected();
    void initializeProfileVelocity();
    void startMotor(quint32 speed, quint8 polarity);
    void stopMotor();
    void pauseMotor();

private slots:
    void onStateChanged(int state);

private:
    bool mConnected;
    QString mPort;
    MotorModbusMaster* mModbusSerial;

    PowerStates mState;
    MotorStates mOperation;
    quint8 mPolarity;
    quint32 mSpeed;

    void sendReadRequest(quint16 address, quint16 dataCount);
    void sendWriteRequest(quint16 address, QByteArray data);
    void processResponse(QModbusResponse m_resp);
    void updatePowerState(quint16 state);
    void stateMachineManager(quint16 command);
    void sendCommand(quint16 command);

};

#endif // MOTORCONTROLLER_H
