#include "sender.h"
#include <QDebug>

Sender::Sender(quint16 localPort, const QHostAddress& destinationAddress, quint16 destinationPort, const QNetworkInterface& interface, QObject *parent)
    : QObject(parent)
    , socket(new QUdpSocket(this))
    , destinationAddress(destinationAddress)
    , destinationPort(destinationPort)
{
    QHostAddress localAddress;
    for (const QNetworkAddressEntry &entry : interface.addressEntries())
    {
        if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol)
        {
            localAddress = entry.ip();
            break;
        }
    }

    if (localAddress.isNull())
    {
        qWarning() << "No IPv4 address found on interface" << interface.humanReadableName();
        return;
    }

    if (!socket->bind(localAddress, localPort, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint))
    {
        qWarning() << "Failed to bind sender:" << socket->errorString();
        return;
    }

    qDebug() << "Sender bound to" << localAddress.toString() << ":" << localPort;
    qDebug() << "Sender bound to remote" << destinationAddress.toString() << ":" << destinationPort;
    valid = true;
}

bool Sender::isValid() const { return valid; }

bool Sender::sendMessage(const DSToRobot& msg)
{
    if (!valid)
        return false;

    uint8_t buffer[DSToRobot_size];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    if (!pb_encode(&stream, DSToRobot_fields, &msg))
    {
        qWarning() << "Failed to serialize protobuf";
        return false;
    }

    qint64 result = socket->writeDatagram(
        reinterpret_cast<const char*>(buffer), stream.bytes_written,
        destinationAddress, destinationPort);

    if (result < 0)
    {
        qWarning() << socket->errorString();
        return false;
    }

    return true;
}

