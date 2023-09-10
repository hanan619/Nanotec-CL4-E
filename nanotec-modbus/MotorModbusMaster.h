#ifndef NANOTECMODBUSMASTER_H
#define NANOTECMODBUSMASTER_H

#include <QModbusRtuSerialMaster>

class MotorModbusMaster : public QModbusRtuSerialMaster
{
    Q_OBJECT
public:
    MotorModbusMaster(QObject *parent = nullptr);

    QModbusReply * sendReadRequest(quint8 nodeId, quint16 index, quint8 subIndex, quint16 startAdd, quint16 dataCount);
    QModbusReply * sendWriteRequest(quint8 nodeId, quint16 index, quint8 subIndex, quint16 startAdd, quint16 dataCount, QByteArray data);

    static constexpr QModbusPdu::FunctionCode ReadEIT { QModbusPdu::FunctionCode(0x2B)};
    static constexpr QModbusPdu::FunctionCode ReadRecord { QModbusPdu::FunctionCode(0x66)};

private:
    bool processPrivateResponse(const QModbusResponse &response, QModbusDataUnit *data) override;
};

#endif // NANOTECMODBUSMASTER_H
