#include "MotorController.h"
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QModbusRtuSerialMaster>
#include <QModbusRequest>
#include <QVariant>
#include <QEventLoop>
#include <QTimer>
#include <QtEndian>
#include <QDebug>

#define CONTROL_WORD    0x6040
#define STATUS_WORD     0x6041

MotorController::MotorController(QString port, QObject *parent) : QObject(parent)
{
    mPort = port;
    mModbusSerial = new MotorModbusMaster();
    mConnected = false;
    mOperation = Init;
    mPolarity = 0xFF;
    mSpeed = 0;

    connect(mModbusSerial, &QModbusClient::stateChanged, this, &MotorController::onStateChanged);
}

MotorController::~MotorController()
{
    if (mModbusSerial)
        mModbusSerial->disconnectDevice();
    delete mModbusSerial;
}

bool MotorController::connectDevice()
{
    if (!mModbusSerial)
        return false;

    if (mModbusSerial->state() != QModbusDevice::ConnectedState) {
        mModbusSerial->setConnectionParameter(QModbusDevice::SerialPortNameParameter, mPort);
        mModbusSerial->setConnectionParameter(QModbusDevice::SerialBaudRateParameter, QSerialPort::Baud19200);
        mModbusSerial->setConnectionParameter(QModbusDevice::SerialParityParameter, QSerialPort::EvenParity);
        mModbusSerial->setConnectionParameter(QModbusDevice::SerialDataBitsParameter, QSerialPort::Data8);
        mModbusSerial->setConnectionParameter(QModbusDevice::SerialStopBitsParameter, QSerialPort::OneStop);

        if (!mModbusSerial->connectDevice()) {
            mConnected = false;
        }
        else
            mConnected = true;
    }

    return mConnected;
}

bool MotorController::isConnected()
{
    return mConnected;
}

void MotorController::initializeProfileVelocity()
{
    if(!mConnected)
        return;

    sendWriteRequest(0x6060, QByteArray::fromHex("03"));        // set mode -> Profile Velocity
    sendWriteRequest(0x6083, QByteArray::fromHex("CA080000"));  // Profile acceleration
    sendWriteRequest(0x6084, QByteArray::fromHex("CA080000"));  // Profile decceleration
    sendWriteRequest(0x6085, QByteArray::fromHex("CA080000"));  // Quick Stop deceleration
    sendWriteRequest(0x60C5, QByteArray::fromHex("CA080000"));  // Max acceleration
    sendWriteRequest(0x60C6, QByteArray::fromHex("CA080000"));  // Max decceleration
    sendWriteRequest(0x605A, QByteArray::fromHex("0600"));  // Quick Stop Operation
}

void MotorController::startMotor(quint32 speed, quint8 polarity)
{
    char bytes[4] = {0,0,0,0};
    if(polarity != mPolarity) {
        mPolarity = polarity;
        bytes[0] = mPolarity ? 0x40 : 0x00;
        sendWriteRequest(0x607E, QByteArray(bytes, 1)); // set polarity
    }

    if(speed != mSpeed) {
        mSpeed = speed;
        qToLittleEndian(mSpeed, bytes);
        sendWriteRequest(0x60FF, QByteArray(bytes, 4)); // set speed
    }

    if(mOperation != MotorStates::Start) {
        mOperation = MotorStates::Start;
        sendReadRequest(STATUS_WORD, 0x02);             // Read State
    }
}

void MotorController::stopMotor()
{
    if(mOperation != MotorStates::Stop) {
        mOperation = MotorStates::Stop;
        sendCommand(0x4D);
    }
}

void MotorController::pauseMotor()
{
    if(mOperation != MotorStates::Pause) {
        mOperation = MotorStates::Pause;
        sendCommand(0x4B);
    }
}

void MotorController::onStateChanged(int state)
{
    if (state == QModbusDevice::UnconnectedState)
        mConnected = false;
    else if (state == QModbusDevice::ConnectedState)
        mConnected = true;
}

void MotorController::sendReadRequest(quint16 address, quint16 dataCount)
{
    if (auto *reply = mModbusSerial->sendReadRequest(0x01, address, 0x00, 0x00, dataCount)) {
        if (!reply->isFinished()) {
            connect(reply, &QModbusReply::finished, this, [this, reply]() {
                if (reply->error() == QModbusDevice::NoError) {
                    processResponse(reply->rawResult());
                } else if (reply->error() == QModbusDevice::ProtocolError) {
                    qDebug() << tr("Write response error: %1 (Modbus exception: 0x%2)")
                                .arg(reply->errorString()).arg(reply->rawResult().exceptionCode(), -1, 16);
                } else if (reply->error() != QModbusDevice::NoError) {
                    qDebug() <<tr("Write response error: %1 (code: 0x%2)").
                               arg(reply->errorString()).arg(reply->error(), -1, 16);
                }

                reply->deleteLater();
            });
        }
        else {
            delete reply;
        }
    } else {
        qDebug() << tr("Write error: ") + mModbusSerial->errorString();
    }
}

void MotorController::sendWriteRequest(quint16 address, QByteArray data)
{
    if (auto *reply = mModbusSerial->sendWriteRequest(0x01, address, 0x00, 0x00, data.length(), data)) {
        if (!reply->isFinished()) {
            connect(reply, &QModbusReply::finished, this, [reply]() {
                if (reply->error() == QModbusDevice::ProtocolError) {
                    qDebug() << tr("Write response error: %1 (Modbus exception: 0x%2)")
                                .arg(reply->errorString()).arg(reply->rawResult().exceptionCode(), -1, 16);
                } else if (reply->error() != QModbusDevice::NoError) {
                    qDebug() <<tr("Write response error: %1 (code: 0x%2)").
                               arg(reply->errorString()).arg(reply->error(), -1, 16);
                }

                reply->deleteLater();
            });
        }
        else {
            delete reply;
        }

    } else {
        qDebug() << tr("Write error: ") + mModbusSerial->errorString();
    }
}

void MotorController::processResponse(QModbusResponse m_resp)
{
    QByteArray byteArray = m_resp.data();
    char* data = byteArray.data();

    if(data[0] != 0x0D || data[1] == 0x01 || data[3] != 0x01) // Unknown Packet, Write Packet or Unknow Node Id
        return;

    if(data[4] == 0x60 && data[5] == 0x41) {
        quint16 value = (data[12] << 8) | data[11];
        updatePowerState(value);
    }
    else if(data[4] == 0x60 && data[5] == 0x40) {
        quint16 value = (data[12] << 8) | data[11];
        stateMachineManager(value);
    }
}

void MotorController::updatePowerState(quint16 state)
{
    state = state & 0x6F;
    switch(state){
    case 0x40: case 0x60:
        mState = SwitchedOff;
        break;
    case 0x21:
        mState = Ready;
        break;
    case 0x23:
        mState = SwitchedOn;
        break;
    case 0x27:
        mState = Running;
        break;
    case 0x07:
        mState = QuickStop;
        break;
    case 0x08: case 0x28:
        mState = Fault;
        break;
    default:
        mState = Unknown;
    }

    qDebug() << "updatePowerState:" << mState;
    if(mState == Unknown){
        sendReadRequest(STATUS_WORD, 0x02);  // Read State
    }
    else {
        sendReadRequest(CONTROL_WORD, 0x02);  // Read Control
    }
}

void MotorController::stateMachineManager(quint16 value)
{
    quint16 command = value;
    switch(mState)
    {
    case SwitchedOff:
        if(mOperation == Start){ // goto Ready
            command = (command & 0xFF7E) | 0x06;
            sendCommand(command);
        }
        break;
    case Ready:
        if(mOperation == Stop){  // goto SwitchedOff
            command = command & 0xFF7D;
            sendCommand(command);
        } else if(mOperation == Start) {  // goto SwitchedOn
            command = (command & 0xFF77) | 0x07;
            sendCommand(command);
        }
        break;
    case SwitchedOn:
        if(mOperation == Stop){  // goto SwitchedOff
            command = command & 0xFF7D;
            sendCommand(command);
        } else if(mOperation == Start) {  // goto Running
            command = (command & 0xFF7F) | 0x0F;
            sendCommand(command);
        }
        break;
    case Running:
        if(mOperation == Stop) {  // goto SwitchedOff
            command = command & 0xFF7D;
            sendCommand(command);
        } else if(mOperation == Pause) {  // goto QuickStop
            command = (command & 0xFF7B) | 0x02;
            sendCommand(command);
        }
        break;
    case QuickStop:
        if(mOperation == Stop) {  // goto SwitchedOff
            command = command & 0xFF7D;
            sendCommand(command);
        } else if(mOperation == Start) {  // goto Running
            command = (command & 0xFF7F) | 0x0F;
            sendCommand(command);
        }
        break;
    case Fault:
        command = command | 0x80;
        sendCommand(command);
        break;
    default:
        sendReadRequest(STATUS_WORD, 0x02);  // Read State
    }
}

void MotorController::sendCommand(quint16 command)
{
    char bytes[2];
    qToLittleEndian(command, bytes);
    sendWriteRequest(CONTROL_WORD, QByteArray(bytes, 2)); // Send Command
    sendReadRequest(STATUS_WORD, 0x02);  // Read State
}
