#include "MotorModbusMaster.h"
#include <QByteArray>
#include <QtDebug>

#define MEI_TYPE_SIZE   1
#define PROTOCOL_SIZE   2
#define ADDRESS_SIZE    6
#define NO_DATA_SIZE    2
#define HEADER_SIZE     MEI_TYPE_SIZE + PROTOCOL_SIZE + ADDRESS_SIZE + NO_DATA_SIZE

#define MIN_READ_SIZE   12
#define MIN_WRITE_SIZE  11

#define SERVER_ADDRESS 0x05


static bool processEITResponse(const QModbusPdu &response, QModbusDataUnit *data)
{
    if (response.dataSize() < HEADER_SIZE)
        return false;

    QByteArray bytes = response.data();
    if(bytes.at(1) == 0x01) // Write response
    {
        if (response.dataSize() < MIN_READ_SIZE){
            qDebug() << "Less than minimum write response size";
            return false;
        }
    }
    else
    {
        if (response.dataSize() < MIN_READ_SIZE){
            qDebug() << "Less than minimum read response size";
            return false;
        }
        quint16 address =  (bytes.at(4) << 8) | bytes.at(5);
        quint16 dataCount = (bytes.at(9) << 8) | bytes.at(10);
        if(dataCount == 0)
            return true;
        if(dataCount == 0x01){
            if(bytes.length() < 13)
                return false;

            quint16 value = (bytes.at(11) << 8) | bytes.at(12);
            if (data)
                *data = {QModbusDataUnit::HoldingRegisters, address, value};
        }
        else {
            if(bytes.length() < HEADER_SIZE + dataCount)
                return false;

            QVector<quint16> values;
            for(int i = 0; i < dataCount; ++i)
            {
                quint16 tmp;
                tmp = (bytes.at(11 + (i*2)) << 8) | bytes.at(12 + (i*2));
                values.append(tmp);
            }
            if (data){
                *data = {QModbusDataUnit::HoldingRegisters, address, values};
            }
        }
    }
    return true;
}

MotorModbusMaster::MotorModbusMaster(QObject *parent) : QModbusRtuSerialMaster(parent)
{

    QModbusResponse::registerDataSizeCalculator(ReadEIT, [](const QModbusResponse &response) {
        if (!response.isValid()){
            qDebug() << "Invalid response";
            return -1;
        }

        if(response.dataSize() < HEADER_SIZE){
            qDebug() << "Incomplete header";
            return -1;
        }

        QByteArray data = response.data();

        if(data.at(1) == 0x01) // Write response
        {
            if (response.dataSize() < MIN_WRITE_SIZE){
                qDebug() << "Less than minimum write response size";
                return -1;
            }
        }
        else
        {
            if (response.dataSize() < MIN_READ_SIZE){
                qDebug() << "Less than minimum read response size";
                return -1;
            }
        }

        quint16 dataSize = (data.at(9) << 8) | data.at(10);
        return HEADER_SIZE + dataSize;
    });

}

QModbusReply *MotorModbusMaster::sendReadRequest(quint8 nodeId, quint16 index, quint8 subIndex, quint16 startAdd, quint16 dataCount)
{
    QByteArray bytes;
    bytes = QByteArray::fromHex("0D0000");
    bytes.resize(HEADER_SIZE);
    bytes[3] = nodeId;
    bytes[4] = (index >> 8) & 0xFF;
    bytes[5] = index & 0xFF;
    bytes[6] = subIndex;
    bytes[7] = (startAdd >> 8) & 0xFF;
    bytes[8] = startAdd & 0xFF;
    bytes[9] = (dataCount >> 8) & 0xFF;
    bytes[10] = dataCount & 0xFF;
    QModbusRequest req(QModbusPdu::FunctionCode(0x2B), bytes);
    qDebug() << "Read" << req;
    return this->sendRawRequest(req, SERVER_ADDRESS);
}

QModbusReply *MotorModbusMaster::sendWriteRequest(quint8 nodeId, quint16 index, quint8 subIndex, quint16 startAdd, quint16 dataCount, QByteArray data)
{
    QByteArray bytes;
    bytes = QByteArray::fromHex("0D0100");
    bytes.resize(HEADER_SIZE);
    bytes[3] = nodeId;
    bytes[4] = (index >> 8) & 0xFF;
    bytes[5] = index & 0xFF;
    bytes[6] = subIndex;
    bytes[7] = (startAdd >> 8) & 0xFF;
    bytes[8] = startAdd & 0xFF;
    bytes[9] = (dataCount >> 8) & 0xFF;
    bytes[10] = dataCount & 0xFF;
    bytes.append(data);
    QModbusRequest req(QModbusPdu::FunctionCode(0x2B), bytes);
    qDebug() << "Write" << req;
    return this->sendRawRequest(req, SERVER_ADDRESS);
}

bool MotorModbusMaster::processPrivateResponse(const QModbusResponse &response, QModbusDataUnit *data)
{
    if (!response.isValid())
        return QModbusClient::processPrivateResponse(response, data);

    if (ReadEIT == response.functionCode())
        return processEITResponse(response, data);

    return QModbusClient::processPrivateResponse(response, data);
}
